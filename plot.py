"""Render telemetry using only Python's standard library."""
import csv
import pathlib
import sys
rows=list(csv.DictReader(open(sys.argv[1],newline='')))
points=' '.join(f'{80+float(r["x_m"])*85:.1f},{445-float(r["y_m"])*85:.1f}' for r in rows)
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="760" height="540" viewBox="0 0 760 540">',
'<rect width="760" height="540" fill="#101c2c"/>',
'<g font-family="sans-serif" fill="#e9f1ff"><text x="40" y="40" font-size="25">RouteBot / simulated delivery mission</text>',
'<text x="40" y="67" font-size="14">Differential-drive controller · 50 Hz simulated clock · meters</text></g>']
for x in range(8): svg.append(f'<path d="M {80+x*85} 95 V 460" stroke="#25374d"/>')
for y in range(5): svg.append(f'<path d="M 65 {445-y*85} H 700" stroke="#25374d"/>')
svg.append(f'<polyline points="{points}" fill="none" stroke="#51dac5" stroke-width="3"/>')
for i,(x,y) in enumerate([(0,0),(6,4),(1,4),(6,0),(3,2),(0,2)]):
    px,py=80+x*85,445-y*85
    svg.append(f'<circle cx="{px}" cy="{py}" r="7" fill="#ffca70"/><text x="{px+12}" y="{py-10}" font-family="sans-serif" fill="white">'+('Depot' if i==0 else f'Station {i}')+'</text>')
blocked=next((r for r in rows if r['state']=='BLOCKED'),None)
if blocked:
    x,y=80+float(blocked['x_m'])*85,445-float(blocked['y_m'])*85
    svg.append(f'<circle cx="{x}" cy="{y}" r="12" fill="none" stroke="#ff8d91" stroke-width="3"/>')
svg.append(f'<text x="40" y="505" font-family="sans-serif" fill="#e9f1ff">{rows[-1]["state"]} at {rows[-1]["time_s"]} s · teal: trajectory · gold: stations · pink: obstacle pause</text></svg>')
pathlib.Path(sys.argv[2]).write_text('\n'.join(svg))
print(sys.argv[2])
