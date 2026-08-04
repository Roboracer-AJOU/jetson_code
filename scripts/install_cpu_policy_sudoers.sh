#!/usr/bin/env bash
# 재시작 후에도 nice(-15 등)가 비밀번호 없이 적용되도록 sudoers 등록
# 한 번만: sudo bash scripts/install_cpu_policy_sudoers.sh
set -euo pipefail

SCRIPT="/home/nvidia/f1tenth_ajou/scripts/apply_cpu_policy.sh"
USER_NAME="${SUDO_USER:-nvidia}"
DROPIN="/etc/sudoers.d/f1tenth-cpu-policy"

if [[ "${EUID}" -ne 0 ]]; then
  echo "root 로 실행하세요: sudo bash $0"
  exit 1
fi

chmod +x "${SCRIPT}"
cat > "${DROPIN}" <<EOF
# F1TENTH CPU policy — passwordless apply for ${USER_NAME}
${USER_NAME} ALL=(root) NOPASSWD: ${SCRIPT}, ${SCRIPT} *
EOF
chmod 440 "${DROPIN}"
visudo -cf "${DROPIN}"

echo "설치 완료: ${DROPIN}"
echo "확인: sudo -n ${SCRIPT} --once"
