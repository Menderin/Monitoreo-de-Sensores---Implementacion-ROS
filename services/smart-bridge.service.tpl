[Unit]
Description=Gateway Monitoring System — ROS-to-MongoDB Bridge
After=smart-agent.service
Requires=smart-agent.service

[Service]
Type=simple
User={{USER}}
ExecStart={{SCRIPTS_DIR}}/start_bridge.sh
Restart=always
RestartSec=5
Environment=PYTHONUNBUFFERED=1
TimeoutStartSec=30

[Install]
WantedBy=multi-user.target
