#!/usr/bin/env bash
set -euo pipefail

cat >&2 <<'EOF'
HATA: launch_phase3_test.sh emekliye ayrıldı; hiçbir ROS prosesi başlatılmadı.

Bu script bayat bir mutlak worktree yolu kullanıyor ve arka plan prosesleri
için güvenilir yaşam döngüsü sağlamıyordu. Donanımsız statik doğrulama:

  ./scripts/verify_workspace.sh

ROS paket doğrulaması:

  ./scripts/verify_workspace.sh --full

Simülasyon gerekiyorsa yalnız ./start_simulation.sh <mod> kullanın.
EOF

exit 64
