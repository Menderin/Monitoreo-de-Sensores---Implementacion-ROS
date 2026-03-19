[Unit]
Description=Gateway Monitoring System — micro-ROS Agent
After=network-online.target NetworkManager.service
Wants=network-online.target

[Service]
Type=simple
User={{USER}}
Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
Environment=HOME={{USER_HOME}}
ExecStart={{SCRIPTS_DIR}}/start_agent.sh
Restart=always
RestartSec=5
TimeoutStartSec=30

[Install]
WantedBy=multi-user.target
