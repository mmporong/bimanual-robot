# md_fontconf: (re)write $CACHE/fonts.conf = system fontconfig + the Pretendard TTF folder, replacing it only
# when the content changes (a copied cache or a moved MOTION_DECK_HOME must not keep pointing elsewhere).
# Sourced by setup.sh and env.sh; soffice calls run with FONTCONFIG_FILE="$CACHE/fonts.conf".
md_fontconf() {
  local fonts="${MOTION_DECK_FONTS:-$CACHE/fonts}" tmp
  [ -d "$fonts" ] || return 0
  tmp="$(mktemp "$CACHE/fonts.conf.XXXXXX")"
  cat > "$tmp" <<CONF
<?xml version="1.0"?>
<!DOCTYPE fontconfig SYSTEM "urn:fontconfig:fonts.dtd">
<fontconfig>
  <include ignore_missing="yes">/etc/fonts/fonts.conf</include>
  <dir>$fonts</dir>
  <cachedir>$CACHE/fontcache</cachedir>
</fontconfig>
CONF
  if cmp -s "$tmp" "$CACHE/fonts.conf"; then rm -f "$tmp"; else mv -f "$tmp" "$CACHE/fonts.conf"; fi
}
