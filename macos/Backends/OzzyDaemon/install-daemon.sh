#!/bin/sh
# Install ozzyd as a LaunchDaemon (system domain, root) and the HAL plug-in that reaches it, from
# this directory's build (make && make hal). CoreAudio's driver helpers run as _coreaudiod and
# can't reach a service in a user's domain, so the daemon has to be a system one. Restarting
# coreaudiod (to load the plug-in) interrupts audio in every app for a moment.
#
#   sudo ./install-daemon.sh             install, or update, and start
#   sudo ./install-daemon.sh uninstall   stop it and remove everything this installs
#
# One driver per device: not alongside OzzyKext, and not with ozzyd-agent.sh's agent loaded.
# For testing for now: the daemon logs a line a second (-v) and takes test requests (-T).

set -e
[ "$(id -u)" = 0 ] || { echo "run as root (sudo)" >&2; exit 1; }
here=$(cd "$(dirname "$0")" && pwd)
label=com.ozzy.daemon
bin=/usr/local/bin/OzzyDaemon   # where macos/install.command looks for the userspace daemon
plist=/Library/LaunchDaemons/$label.plist
logdir=/Library/Logs/Ozzy
hal=/Library/Audio/Plug-Ins/HAL/OzzyHAL.driver

if [ "$1" = uninstall ]; then
	launchctl bootout "system/$label" 2>/dev/null || true
	rm -f "$plist" "$bin"
	rm -rf "$hal"
	killall coreaudiod 2>/dev/null || true
	echo "removed $bin, $plist, $hal (the log stays in $logdir)"
	exit 0
fi

[ -x "$here/ozzyd" ] && [ -d "$here/build/OzzyHAL.driver" ] || { echo "build first: make && make hal" >&2; exit 1; }
[ -d /Library/Extensions/OzzyKext.kext ] && { echo "OzzyKext is installed: one driver per device" >&2; exit 1; }

launchctl bootout "system/$label" 2>/dev/null || true
mkdir -p /usr/local/bin "$logdir"
install -m 755 "$here/ozzyd" "$bin"
cat > "$plist" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
	<key>Label</key>
	<string>$label</string>
	<key>ProgramArguments</key>
	<array>
		<string>$bin</string>
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
	<key>KeepAlive</key>
	<true/>
	<key>ProcessType</key>
	<string>Interactive</string>
	<key>StandardOutPath</key>
	<string>$logdir/ozzyd.log</string>
	<key>StandardErrorPath</key>
	<string>$logdir/ozzyd.log</string>
</dict>
</plist>
EOF
chown root:wheel "$plist"
chmod 644 "$plist"
launchctl bootstrap system "$plist"

rm -rf "$hal"
cp -R "$here/build/OzzyHAL.driver" "$hal"
chown -R root:wheel "$hal"
chmod -R 755 "$hal"
killall coreaudiod 2>/dev/null || true   # launchd starts it again at once, with the plug-in
echo "installed $bin ($label, log $logdir/ozzyd.log) and $hal"
