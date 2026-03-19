[Unit]
Description=Gateway Monitoring System — ROS-to-MongoDB Bridge
After=smart-agent.service
Requires=smart-agent.service

[Service]
Type=simple
User={{USER}}
Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
Environment=HOME={{USER_HOME}}
Environment=PYTHONUNBUFFERED=1
ExecStart={{SCRIPTS_DIR}}/start_bridge.sh
Restart=always
RestartSec=5
TimeoutStartSec=30

[Install]
WantedBy=multi-user.target
