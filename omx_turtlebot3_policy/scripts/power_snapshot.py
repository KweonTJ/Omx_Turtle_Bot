#!/usr/bin/env python3
"""Read-only USB/power snapshot. Never starts cameras, ROS control or serial IO."""
import datetime
import glob
import json
from pathlib import Path
import re
import subprocess


def command(args):
    try:
        r = subprocess.run(args, text=True, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, timeout=15, check=False)
        return dict(exit_code=r.returncode, output=r.stdout)
    except (OSError, subprocess.TimeoutExpired) as e:
        return dict(error=str(e))


def main():
    result = dict(time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  read_only=True, actual_usb_power_measured=False)
    for key, args in {
        'usb_tree':['lsusb','-t'], 'usb_devices':['lsusb'],
        'astra_descriptors':['lsusb','-v','-d','2bc5:0401'],
        'hub_descriptors':['lsusb','-v','-d','0bda:5489'],
        'power_mode':['nvpmodel','-q'],
        'kernel_log':['journalctl','-k','-b','--no-pager','-o','short-monotonic'],
        'uptime':['uptime'], 'processes':['ps','-eo','pid,comm'],
    }.items():
        result[key] = command(args)
    log = result['kernel_log'].get('output', '')
    matched = [line for line in log.splitlines() if re.search(
        r'usb|under.?voltage|over.?current|soctherm|vdd|ina3221|throttl|ttyACM|ttyUSB', line, re.I)]
    result['kernel_log']['output'] = '\n'.join(matched[-250:])
    result['kernel_log']['filter'] = 'USB, power, thermal and serial; last 250 matched lines'
    result['kernel_log']['matched_lines'] = len(matched)
    result['hwmon'] = {}
    for directory in glob.glob('/sys/class/hwmon/hwmon*'):
        fields = {}
        for name in ['name', '*_label', '*_input', '*_alarm', 'oc*_event_cnt']:
            for path in Path(directory).glob(name):
                try:
                    fields[path.name] = path.read_text().strip()
                except OSError as e:
                    fields[path.name] = str(e)
        result['hwmon'][directory] = fields
    result['serial_by_id'] = {str(p):str(p.resolve()) for p in Path('/dev/serial/by-id').glob('*')}
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
