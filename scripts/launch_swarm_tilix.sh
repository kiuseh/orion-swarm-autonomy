#!/usr/bin/env bash

set -euo pipefail

fail() {
    printf 'Error: %s\n' "$*" >&2
    exit 1
}

warn() {
    printf 'Warning: %s\n' "$*" >&2
}

json_escape() {
    local value=${1-}
    value=${value//\\/\\\\}
    value=${value//\"/\\\"}
    value=${value//$'\n'/\\n}
    value=${value//$'\r'/\\r}
    value=${value//$'\t'/\\t}
    printf '%s' "$value"
}

bash_ic_command() {
    local payload="${1}; exec bash"
    printf 'bash -ic %q' "$payload"
}

require_command() {
    local command_name=$1
    local error_message=$2

    if ! command -v "$command_name" >/dev/null 2>&1; then
        fail "$error_message"
    fi
}

get_profile_uuid() {
    local uuid

    uuid="$(dconf read /com/gexperts/Tilix/profiles/default 2>/dev/null | tr -d "'" || true)"
    if [[ $uuid =~ ^[0-9a-fA-F-]{36}$ ]]; then
        printf '%s\n' "$uuid"
        return 0
    fi

    uuid="$(
        dconf dump /com/gexperts/Tilix/ 2>/dev/null \
            | sed -nE 's/^\[profiles\/([0-9a-fA-F-]{36})\]$/\1/p' \
            | head -n 1 \
            || true
    )"
    if [[ $uuid =~ ^[0-9a-fA-F-]{36}$ ]]; then
        printf '%s\n' "$uuid"
        return 0
    fi

    return 1
}

regex_escape() {
    printf '%s\n' "$1" | sed -e 's/[][(){}.^$?*+|/\\-]/\\&/g'
}

window_id_for_xdotool() {
    local window_id=$1
    printf '%d\n' "$window_id" 2>/dev/null || printf '%s\n' "$window_id"
}

find_window_id_by_class() {
    local window_list=$1
    local class_name=$2

    awk -v class_name="$class_name" '$8 == class_name { print $1; exit }' <<<"$window_list"
}

find_window_id_by_title() {
    local window_list=$1
    local title=$2
    local title_pattern

    title_pattern="$(regex_escape "$title")"
    grep -E "[[:space:]]${title_pattern}$" <<<"$window_list" | head -n 1 | awk '{ print $1 }'
}

window_is_hidden() {
    local window_id=$1

    xprop -id "$window_id" _NET_WM_STATE 2>/dev/null | grep -q '_NET_WM_STATE_HIDDEN'
}

ensure_window_visible() {
    local window_id=$1
    local xdotool_window_id

    if ! window_is_hidden "$window_id"; then
        return 0
    fi

    xdotool_window_id="$(window_id_for_xdotool "$window_id")"
    xdotool windowmap "$xdotool_window_id" >/dev/null 2>&1 || true
    xdotool windowactivate --sync "$xdotool_window_id" >/dev/null 2>&1 || true
    wmctrl -i -a "$window_id" >/dev/null 2>&1 || true
}

move_and_resize_window() {
    local window_id=$1
    local x=$2
    local y=$3
    local width=$4
    local height=$5

    wmctrl -i -r "$window_id" -b remove,maximized_vert,maximized_horz >/dev/null 2>&1 || true
    wmctrl -i -r "$window_id" -e "0,${x},${y},${width},${height}" >/dev/null 2>&1 || true
}

set_window_above() {
    local window_id=$1

    wmctrl -i -r "$window_id" -b add,above >/dev/null 2>&1 || true
}

apply_window_layout() {
    local window_id=$1
    local x=$2
    local y=$3
    local width=$4
    local height=$5
    local keep_above=$6

    [[ -n $window_id ]] || return 0

    ensure_window_visible "$window_id"
    move_and_resize_window "$window_id" "$x" "$y" "$width" "$height"

    if [[ $keep_above == "1" ]]; then
        set_window_above "$window_id"
    fi
}

maybe_apply_window_layout() {
    local window_id=$1
    local x=$2
    local y=$3
    local width=$4
    local height=$5
    local keep_above=$6
    local last_applied_window_id=${7-}

    if [[ -z $window_id ]]; then
        printf '\n'
        return 0
    fi

    if [[ $window_id == "$last_applied_window_id" ]]; then
        printf '%s\n' "$last_applied_window_id"
        return 0
    fi

    apply_window_layout "$window_id" "$x" "$y" "$width" "$height" "$keep_above"
    printf '%s\n' "$window_id"
}

find_qgroundcontrol_desktop_file() {
    local desktop_file

    for desktop_file in \
        "$HOME/.local/share/applications"/appimagekit_*QGroundControl*.desktop \
        "$HOME/.local/share/applications"/*QGroundControl*.desktop \
        "/usr/share/applications"/*QGroundControl*.desktop
    do
        [[ -f $desktop_file ]] || continue
        printf '%s\n' "$desktop_file"
        return 0
    done

    return 1
}

resolve_qgroundcontrol_launcher() {
    local desktop_file
    local exec_line
    local fallback_appimage

    desktop_file="$(find_qgroundcontrol_desktop_file || true)"
    if [[ -n $desktop_file ]]; then
        exec_line="$(
            sed -n 's/^Exec=//p' "$desktop_file" | head -n 1
        )"
        exec_line="$(
            printf '%s\n' "$exec_line" \
                | sed -E 's/[[:space:]]+%[[:alpha:]]//g; s/[[:space:]]+$//'
        )"
        if [[ -n $exec_line ]]; then
            printf '%s\n' "$exec_line"
            return 0
        fi
    fi

    fallback_appimage="${SWARM_QGROUNDCONTROL_APPIMAGE:-$HOME/Uygulamalar/QGroundControl-x86_64_e39cc090356a6b6f41df88eff8cfbfe9.AppImage}"
    if [[ -x $fallback_appimage ]]; then
        printf '%q\n' "$fallback_appimage"
        return 0
    fi

    return 1
}

launch_qgroundcontrol() {
    local launcher_command=$1

    [[ -n $launcher_command ]] || return 1
    nohup bash -lc "$launcher_command" >/dev/null 2>&1 &
}

layout_monitor() {
    local qgroundcontrol_launcher
    local qgroundcontrol_launch_attempted=0
    local duration_seconds
    local poll_interval_seconds
    local start_seconds
    local deadline_seconds
    local window_list
    local tilix_window_id
    local qgroundcontrol_window_id
    local gazebo_window_id
    local interface_window_id
    local camera_lider_window_id
    local camera_sol_window_id
    local tilix_layout_applied_to=""
    local qgroundcontrol_layout_applied_to=""
    local gazebo_layout_applied_to=""
    local interface_layout_applied_to=""
    local camera_lider_layout_applied_to=""
    local camera_sol_layout_applied_to=""

    require_command wmctrl "wmctrl bulunamadi. Pencere yerlesimini uygulamak icin wmctrl gerekli."
    require_command xdotool "xdotool bulunamadi. Pencereleri gorunur hale getirmek icin xdotool gerekli."

    qgroundcontrol_launcher="$(resolve_qgroundcontrol_launcher || true)"
    if [[ -z $qgroundcontrol_launcher ]]; then
        warn "QGroundControl baslatma komutu bulunamadi. Acik pencere varsa yine yerlestirilecek."
    fi

    duration_seconds="${SWARM_LAYOUT_DURATION:-60}"
    poll_interval_seconds="${SWARM_LAYOUT_POLL_INTERVAL:-0.25}"
    start_seconds=$SECONDS
    deadline_seconds=$((start_seconds + duration_seconds))

    while (( SECONDS < deadline_seconds )); do
        window_list="$(wmctrl -lGpx 2>/dev/null || true)"

        tilix_window_id="$(find_window_id_by_class "$window_list" "tilix.Tilix" || true)"
        qgroundcontrol_window_id="$(
            find_window_id_by_class "$window_list" "QGroundControl.QGroundControl" || true
        )"
        gazebo_window_id="$(find_window_id_by_title "$window_list" "Gazebo Sim" || true)"
        interface_window_id="$(
            find_window_id_by_title "$window_list" "YKI Drone Kontrol Arayüzü" || true
        )"
        camera_lider_window_id="$(
            find_window_id_by_title "$window_list" "IHA KAMERA LIDER" || true
        )"
        camera_sol_window_id="$(
            find_window_id_by_title "$window_list" "IHA KAMERA SOL" || true
        )"

        if [[ -z $qgroundcontrol_window_id && $qgroundcontrol_launch_attempted -eq 0 ]]; then
            if [[ -n $qgroundcontrol_launcher ]]; then
                launch_qgroundcontrol "$qgroundcontrol_launcher" || true
                qgroundcontrol_launch_attempted=1
            fi
        fi

        tilix_layout_applied_to="$(
            maybe_apply_window_layout \
                "$tilix_window_id" -52 -46 1638 332 1 "$tilix_layout_applied_to"
        )"
        qgroundcontrol_layout_applied_to="$(
            maybe_apply_window_layout \
                "$qgroundcontrol_window_id" 1215 86 719 1124 0 "$qgroundcontrol_layout_applied_to"
        )"
        gazebo_layout_applied_to="$(
            maybe_apply_window_layout \
                "$gazebo_window_id" 14 279 1200 930 0 "$gazebo_layout_applied_to"
        )"
        interface_layout_applied_to="$(
            maybe_apply_window_layout \
                "$interface_window_id" 453 810 503 395 0 "$interface_layout_applied_to"
        )"
        camera_lider_layout_applied_to="$(
            maybe_apply_window_layout \
                "$camera_lider_window_id" 1454 933 480 277 1 "$camera_lider_layout_applied_to"
        )"
        camera_sol_layout_applied_to="$(
            maybe_apply_window_layout \
                "$camera_sol_window_id" 974 937 480 273 1 "$camera_sol_layout_applied_to"
        )"

        sleep "$poll_interval_seconds"
    done
}

spawn_layout_monitor() {
    local script_path=$1
    nohup bash "$script_path" --monitor-layout >/dev/null 2>&1 &
}

terminal_json() {
    local command=$1
    local title=${2:-$1}
    local override_command_json
    local title_json

    override_command_json="$(json_escape "$(bash_ic_command "$command")")"
    title_json="$(json_escape "$title")"

    cat <<EOF
{
  "type": "Terminal",
  "profile": "$PROFILE_UUID",
  "directory": "$WORKDIR_JSON",
  "title": "$title_json",
  "overrideCommand": "$override_command_json"
}
EOF
}

write_three_column_session() {
    local file=$1
    local session_name=$2
    local left_command=$3
    local middle_command=$4
    local right_command=$5
    local session_name_json

    session_name_json="$(json_escape "$session_name")"

    cat >"$file" <<EOF
{
  "version": "1.0",
  "name": "$session_name_json",
  "type": "Session",
  "width": $SESSION_WIDTH,
  "height": $SESSION_HEIGHT,
  "synchronizedInput": false,
  "child": {
    "type": "Paned",
    "orientation": 0,
    "position": 33,
    "ratio": 0.3333333333,
    "child1": $(terminal_json "$left_command"),
    "child2": {
      "type": "Paned",
      "orientation": 0,
      "position": 50,
      "ratio": 0.5,
      "child1": $(terminal_json "$middle_command"),
      "child2": $(terminal_json "$right_command")
    }
  }
}
EOF
}

write_two_column_session() {
    local file=$1
    local session_name=$2
    local left_command=$3
    local right_command=$4
    local session_name_json

    session_name_json="$(json_escape "$session_name")"

    cat >"$file" <<EOF
{
  "version": "1.0",
  "name": "$session_name_json",
  "type": "Session",
  "width": $SESSION_WIDTH,
  "height": $SESSION_HEIGHT,
  "synchronizedInput": false,
  "child": {
    "type": "Paned",
    "orientation": 0,
    "position": 50,
    "ratio": 0.5,
    "child1": $(terminal_json "$left_command"),
    "child2": $(terminal_json "$right_command")
  }
}
EOF
}

write_single_terminal_session() {
    local file=$1
    local session_name=$2
    local command=$3
    local session_name_json

    session_name_json="$(json_escape "$session_name")"

    cat >"$file" <<EOF
{
  "version": "1.0",
  "name": "$session_name_json",
  "type": "Session",
  "width": $SESSION_WIDTH,
  "height": $SESSION_HEIGHT,
  "synchronizedInput": false,
  "child": $(terminal_json "$command")
}
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
SCRIPT_PATH="$SCRIPT_DIR/$(basename -- "${BASH_SOURCE[0]}")"

if [[ ${1-} == "--monitor-layout" ]]; then
    layout_monitor
    exit 0
fi

require_command dconf "dconf bulunamadi. Tilix profil bilgisini okumak icin dconf gerekli."
require_command wmctrl "wmctrl bulunamadi. Launch sonrasinda pencere yerlesimini uygulamak icin wmctrl gerekli."
require_command xdotool "xdotool bulunamadi. Launch sonrasinda pencere gorunurlugu icin xdotool gerekli."

TILIX_BIN="${TILIX_BIN:-tilix}"
if ! command -v "$TILIX_BIN" >/dev/null 2>&1; then
    fail "Tilix calistirilamadi. TILIX_BIN='$TILIX_BIN' gecerli bir komut olmali."
fi

REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." >/dev/null 2>&1 && pwd)"
WORKDIR="$REPO_ROOT"
WORKDIR_JSON="$(json_escape "$WORKDIR")"
SESSION_WIDTH="${SWARM_TILIX_WIDTH:-1600}"
SESSION_HEIGHT="${SWARM_TILIX_HEIGHT:-900}"
KEEP_TEMP="${SWARM_TILIX_KEEP_TEMP:-0}"

PROFILE_UUID="$(get_profile_uuid)" \
    || fail "Tilix profil UUID'si bulunamadi. Tilix'i en az bir kez acip tekrar deneyin."

TMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/swarm-tilix.XXXXXX")"
cleanup() {
    if [[ $KEEP_TEMP == "1" ]]; then
        printf 'Tilix session dosyalari korunuyor: %s\n' "$TMP_DIR" >&2
        return
    fi
    rm -rf -- "$TMP_DIR"
}
trap cleanup EXIT

TAB1_SESSION="$TMP_DIR/01_bay_drone.json"
TAB2_SESSION="$TMP_DIR/02_drone_control.json"
TAB3_SESSION="$TMP_DIR/03_g_isleme.json"
TAB4_SESSION="$TMP_DIR/04_arayuz.json"

write_three_column_session "$TAB1_SESSION" "Bay Drone" "baydrone1" "baydrone2" "baydrone3"
write_three_column_session "$TAB2_SESSION" "Drone Control" "drone14541" "drone14542" "drone14543"
write_two_column_session "$TAB3_SESSION" "G Isleme" "gisleme1" "gisleme2"
write_single_terminal_session "$TAB4_SESSION" "Yer Kontrol" "ros2 run arayuz_pkg qgc_interface"

"$TILIX_BIN" -s "$TAB1_SESSION" -s "$TAB2_SESSION" -s "$TAB3_SESSION" -s "$TAB4_SESSION"
spawn_layout_monitor "$SCRIPT_PATH"

# Tilix mevcut instance'a komutlari asenkron aktarirsa session dosyalari
# hemen silinmesin diye kisa bir bekleme birakiyoruz.
if [[ $KEEP_TEMP != "1" ]]; then
    sleep "${SWARM_TILIX_CLEANUP_DELAY:-1}"
fi
