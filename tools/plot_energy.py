import csv
from pathlib import Path

rows = list(csv.DictReader(Path('build/integrator_energy.csv').open()))
colors = {'Symplectic': '#38bdf8', 'Verlet': '#34d399', 'RK4': '#fb923c'}
parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="500" viewBox="0 0 1000 500">',
         '<rect width="1000" height="500" fill="#111827"/>',
         '<text x="60" y="35" fill="white" font-family="sans-serif" font-size="20">Oscillator energy error — ω=10, dt=0.01, 1000 seconds</text>',
         '<path d="M60 60V430H950" fill="none" stroke="#64748b"/>']
for policy, color in colors.items():
    data = [(float(r['time']), float(r['relative_error'])) for r in rows if r['policy'] == policy]
    assert len(data) == 1001 and all(abs(e) < 0.1 for _, e in data)
    points = ' '.join(f'{60+t/1000*890:.2f},{245-e/0.06*175:.2f}' for t, e in data)
    parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.5"/>')
    i = list(colors).index(policy)
    parts.append(f'<text x="{100+i*250}" y="475" fill="{color}" font-family="sans-serif">{policy}</text>')
parts += ['<text x="60" y="450" fill="white">0 s</text><text x="900" y="450" fill="white">1000 s</text>',
          '<text x="5" y="75" fill="white">+6%</text><text x="5" y="250" fill="white">0</text><text x="5" y="425" fill="white">−6%</text>', '</svg>']
Path('build/integrator_energy.svg').write_text('\n'.join(parts), encoding='utf-8')
print('Generated build/integrator_energy.svg from 3003 measured samples')
