[Unit]
Description=Gateway Monitoring System — Telegram Alert Daemon
After=network-online.target NetworkManager-wait-online.service smart-bridge.service
Wants=network-online.target
Requires=smart-bridge.service

[Service]
Type=simple
User={{USER}}
WorkingDirectory={{REPO_DIR}}/database
Environment=HOME={{USER_HOME}}
Environment=PYTHONUNBUFFERED=1
ExecStart=/usr/bin/python3 -u {{SCRIPTS_DIR}}/telegram/smart_alerter.py
Restart=on-failure
RestartSec=30
StartLimitIntervalSec=300
StartLimitBurst=5
TimeoutStartSec=30

[Install]
WantedBy=multi-user.target
