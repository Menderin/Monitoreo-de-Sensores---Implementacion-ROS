[Unit]
Description=Gateway Monitoring System — Telegram Alert Daemon
After=smart-bridge.service
Requires=smart-bridge.service

[Service]
Type=simple
User={{USER}}
WorkingDirectory={{REPO_DIR}}/database
ExecStart=/usr/bin/python3 -u {{SCRIPTS_DIR}}/telegram/smart_alerter.py
Restart=on-failure
RestartSec=10
Environment=PYTHONUNBUFFERED=1
TimeoutStartSec=30

[Install]
WantedBy=multi-user.target
