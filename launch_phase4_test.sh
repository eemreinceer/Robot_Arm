#!/usr/bin/env bash
set -euo pipefail

cat >&2 <<'EOF'
HATA: launch_phase4_test.sh emekliye ayrıldı; hiçbir sim veya ROS prosesi
başlatılmadı.

Eski script bayat bir worktree yolu, ad-hoc headless launch ve cleanup'sız arka
plan prosesleri kullanıyordu. Donanımsız statik doğrulama:

  ./scripts/verify_workspace.sh

ROS paket doğrulaması:

  ./scripts/verify_workspace.sh --full

Simülasyon gerekiyorsa yalnız ./start_simulation.sh <mod> kullanın; aynı anda
tek sim açın ve iş sonunda bütün çocuk prosesleri kapatın.
EOF

exit 64
