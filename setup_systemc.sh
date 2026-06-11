PROJECT_DIR="/home/g1/pap/final"
echo "⚙️ Configurando serviço systemd..."
sudo tee /etc/systemd/system/tractor-control.service > /dev/null <<EOF
[Unit]
Description=Smart SafeTech Tractor Control System
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$PROJECT_DIR
Environment=PATH=$PROJECT_DIR/venv/bin
ExecStart=$PROJECT_DIR/venv/bin/python main.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "🚀 Ativando serviço..."
sudo systemctl daemon-reload
sudo systemctl enable tractor-control.service
