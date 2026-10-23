#!/bin/sh
# Run ozzyd as a LaunchAgent in this user's launchd domain: no admin, nothing installed, gone at
# logout or with "unload". launchd registers its Mach service (com.ozzy.daemon) from the plist,
# which ozzyd needs to listen on it. For testing with tools/macos-harness/ozzyd-client: CoreAudio's
# driver helpers run as _coreaudiod and can't reach a user-domain service, so OzzyHAL needs ozzyd
# as a LaunchDaemon in the system domain instead.
#
#   ozzyd-agent.sh load [log]   start ./ozzyd -v -T (stats every second, test requests on), its
#                               output to log (default /tmp/ozzyd.log, appended)
#   ozzyd-agent.sh unload       stop it (SIGTERM: it stops the engines and prints their stats)
#   ozzyd-agent.sh status

set -e
here=$(cd "$(dirname "$0")" && pwd)
label=com.ozzy.daemon
domain="gui/$(id -u)"
plist="$here/build/$label.plist"

case "$1" in
load)
	log=${2:-/tmp/ozzyd.log}
	[ -x "$here/ozzyd" ] || { echo "build it first: make -C $here" >&2; exit 1; }
	mkdir -p "$here/build"
	cat > "$plist" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>Label</key>
	<string>$label</string>
	<key>ProgramArguments</key>
	<array>
		<string>$here/ozzyd</string>
		<string>-v</string>
		<string>-T</string>
	</array>
	<key>MachServices</key>
	<dict>
		<key>$label</key>
		<true/>
	</dict>
	<key>RunAtLoad</key>
	<true/>
	<key>ProcessType</key>
	<string>Interactive</string>
	<key>StandardOutPath</key>
	<string>$log</string>
	<key>StandardErrorPath</key>
	<string>$log</string>
</dict>
</plist>
EOF
	launchctl bootstrap "$domain" "$plist"
	echo "loaded $label in $domain; log: $log"
	;;
unload)
	launchctl bootout "$domain/$label"
	;;
status)
	launchctl print "$domain/$label"
	;;
*)
	echo "usage: $0 load [log] | unload | status" >&2
	exit 2
	;;
esac
