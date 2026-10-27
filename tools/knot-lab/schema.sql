-- Knot Lab backend (Supabase / Postgres).
-- Paste into the Supabase SQL editor and run once; running it again is safe.
--
-- The tables are closed to the public (anon) key. Visitors can only call the
-- three functions at the bottom: knot_login, knot_submit and knot_leaderboard.

create extension if not exists pgcrypto with schema extensions;

create table if not exists public.knot_players (
  username      text primary key check (username ~ '^[A-Za-z0-9_-]{3,20}$'),
  pin_hash      text not null,                 -- bcrypt hash of the 4-digit PIN
  created_at    timestamptz not null default now(),
  failed_logins int not null default 0,
  locked_until  timestamptz
);
create unique index if not exists knot_players_username_lower on public.knot_players (lower(username));

-- One row per successfully untied knot. `data` is the recording: base64 of a
-- gzip-compressed binary file, see tools/knot-lab/README.md.
create table if not exists public.knot_episodes (
  id         bigint generated always as identity primary key,
  username   text not null references public.knot_players (username) on delete cascade,
  created_at timestamptz not null default now(),
  knot       text not null,
  crossings  int not null,
  seconds    real not null,
  grasps     int not null,
  misses     int not null,
  hints      int not null,
  points     int not null,
  meta       jsonb not null,
  data       text not null
);
create index if not exists knot_episodes_username on public.knot_episodes (username);
create index if not exists knot_episodes_created on public.knot_episodes (created_at);

alter table public.knot_players enable row level security;
alter table public.knot_episodes enable row level security;
revoke all on public.knot_players, public.knot_episodes from anon, authenticated;

-- Shared PIN check. Returns the player, or null with `err` set. Wrong PINs
-- are counted; five in a row lock the name for 15 minutes.
create or replace function public.knot_check_pin(p_username text, p_pin text, out player public.knot_players, out err text)
language plpgsql security definer set search_path = public, extensions as $$
begin
  select * into player from public.knot_players where lower(username) = lower(p_username) for update;
  if not found then err := 'unknown'; return; end if;
  if player.locked_until is not null and player.locked_until > now() then err := 'locked'; player := null; return; end if;
  if player.pin_hash <> crypt(p_pin, player.pin_hash) then
    update public.knot_players
      set failed_logins = failed_logins + 1,
          locked_until = case when failed_logins + 1 >= 5 then now() + interval '15 minutes' end
      where username = player.username;
    err := 'wrong_pin'; player := null; return;
  end if;
  update public.knot_players set failed_logins = 0, locked_until = null where username = player.username;
end $$;
revoke all on function public.knot_check_pin(text, text) from public, anon, authenticated;

-- Sign in, or create the player on first use.
create or replace function public.knot_login(p_username text, p_pin text)
returns json language plpgsql security definer set search_path = public, extensions as $$
declare r record;
begin
  if p_username is null or p_username !~ '^[A-Za-z0-9_-]{3,20}$' then return json_build_object('ok', false, 'error', 'bad_username'); end if;
  if p_pin is null or p_pin !~ '^[0-9]{4}$' then return json_build_object('ok', false, 'error', 'bad_pin'); end if;
  select * into r from public.knot_check_pin(p_username, p_pin);
  if r.err = 'unknown' then
    insert into public.knot_players (username, pin_hash) values (p_username, crypt(p_pin, gen_salt('bf', 8)));
    return json_build_object('ok', true, 'new', true, 'username', p_username);
  end if;
  if r.err is not null then return json_build_object('ok', false, 'error', r.err); end if;
  return json_build_object('ok', true, 'new', false, 'username', (r.player).username);
end $$;

-- Save one untied knot. Points are computed here: 10 per crossing, up to the
-- same again for speed, halved if a hint was used.
create or replace function public.knot_submit(p_username text, p_pin text, p_meta jsonb, p_data text)
returns json language plpgsql security definer set search_path = public, extensions as $$
declare
  r record; v_cross int; v_sec real; v_grasps int; v_misses int; v_hints int; v_limit real; v_base real; v_points int; v_id bigint;
begin
  if p_username is null or p_pin is null or p_pin !~ '^[0-9]{4}$' then return json_build_object('ok', false, 'error', 'bad_login'); end if;
  select * into r from public.knot_check_pin(p_username, p_pin);
  if r.err is not null then return json_build_object('ok', false, 'error', case r.err when 'unknown' then 'bad_login' else r.err end); end if;
  if exists (select 1 from public.knot_episodes where username = (r.player).username and created_at > now() - interval '5 seconds') then
    return json_build_object('ok', false, 'error', 'too_fast');
  end if;
  if p_data is null or length(p_data) > 6000000 or length(p_data) < 100 then return json_build_object('ok', false, 'error', 'bad_data'); end if;
  begin
    v_cross  := (p_meta ->> 'crossings')::int;
    v_sec    := (p_meta ->> 'seconds')::real;
    v_grasps := (p_meta ->> 'grasps')::int;
    v_misses := (p_meta ->> 'misses')::int;
    v_hints  := (p_meta ->> 'hints')::int;
  exception when others then
    return json_build_object('ok', false, 'error', 'bad_meta');
  end;
  if v_cross is null or v_cross not between 3 and 9 or v_sec is null or v_sec not between 1 and 900
     or v_grasps is null or v_grasps not between 1 and 2000 or v_misses is null or v_misses not between 0 and 2000
     or v_hints is null or v_hints not between 0 and 500 or (p_meta ->> 'knot') !~ '^[3-9]_[0-9]{1,2}$' then
    return json_build_object('ok', false, 'error', 'bad_meta');
  end if;
  v_limit  := greatest(180, 60 * v_cross);
  v_base   := 10 * v_cross;
  v_points := round((v_base + v_base * greatest(0, 1 - v_sec / v_limit)) * case when v_hints > 0 then 0.5 else 1 end);
  insert into public.knot_episodes (username, knot, crossings, seconds, grasps, misses, hints, points, meta, data)
    values ((r.player).username, p_meta ->> 'knot', v_cross, v_sec, v_grasps, v_misses, v_hints, v_points, p_meta, p_data)
    returning id into v_id;
  return json_build_object('ok', true, 'id', v_id, 'points', v_points);
end $$;

-- Totals per player, best first, plus the number of runs collected so far.
create or replace function public.knot_leaderboard(p_limit int default 20)
returns json language sql security definer set search_path = public as $$
  select json_build_object(
    'runs', (select count(*) from public.knot_episodes),
    'players', (select count(distinct username) from public.knot_episodes),
    'top', coalesce((select json_agg(t) from (
      select username, sum(points)::int as points, count(*)::int as untied, max(crossings) as hardest,
             round(min(seconds)::numeric, 1) as best_seconds
      from public.knot_episodes group by username
      order by sum(points) desc, min(seconds) asc
      limit least(greatest(coalesce(p_limit, 20), 1), 100)) t), '[]'::json));
$$;

revoke all on function public.knot_login(text, text), public.knot_submit(text, text, jsonb, text), public.knot_leaderboard(int) from public;
grant execute on function public.knot_login(text, text), public.knot_submit(text, text, jsonb, text), public.knot_leaderboard(int) to anon, authenticated;
