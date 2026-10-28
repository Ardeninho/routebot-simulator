import csv
import io
import math
import subprocess

def run(scenario):
    p = subprocess.run(['./build/routebot', scenario], capture_output=True, text=True, timeout=15)
    assert p.returncode == (1 if scenario in ('estop', 'dropout') else 0), p.stderr
    rows = list(csv.DictReader(io.StringIO(p.stdout)))
    for r in rows:
        assert all(math.isfinite(float(r[k])) for k in ('x_m','y_m','left_mps','right_mps'))
        assert max(abs(float(r[k])) for k in ('left_mps','right_mps')) <= .8
        if r['state'] in ('BLOCKED','FAULT','COMPLETE'):
            assert float(r['left_mps']) == float(r['right_mps']) == 0
    return rows
normal = run('normal')
assert normal[-1]['state'] == 'COMPLETE'
assert math.hypot(float(normal[-1]['x_m']),float(normal[-1]['y_m'])) < .06
assert set(int(r['target']) for r in normal) == set(range(1,8))
obstacle = run('obstacle')
blocked = [r for r in obstacle if r['state'] == 'BLOCKED']
assert len(blocked) == 150
assert len({(r['x_m'],r['y_m']) for r in blocked}) == 1
assert obstacle[-1]['state'] == 'COMPLETE'
assert abs(float(obstacle[-1]['time_s']) - float(normal[-1]['time_s']) - 3) < .021
for scenario, fault in [('estop','2'),('dropout','1')]:
    rows=run(scenario)
    assert rows[-1]['state']=='FAULT' and rows[-1]['fault']==fault
    assert float(rows[-1]['time_s'])==3
print('Four end-to-end scenarios passed; commands bounded, stops enforced, route completed.')
