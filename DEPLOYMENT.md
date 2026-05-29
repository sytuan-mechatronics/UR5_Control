# Deployment Guide - PC2 UR5 Server

Hướng dẫn deploy server PC2 trên production.

## Lựa chọn: Bare Metal vs Docker

### Option 1: Bare Metal (Linux/Windows)

**Ưu điểm:**
- Trực tiếp kiểm soát dependencies
- Dễ debug
- Tốt cho development

**Nhược điểm:**
- Phụ thuộc OS
- Cài Orbbec SDK phức tạp

### Option 2: Docker (Recommended)

**Ưu điểm:**
- Portable, consistent environment
- Dễ scale
- Dễ deploy trên cloud

**Nhược điểm:**
- Phải pass-through USB cho camera
- Hơi overhead về tài nguyên

---

## Option 1: Bare Metal Deployment

### Linux (Ubuntu 20.04+)

#### 1. Prerequisites
```bash
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-venv
sudo apt-get install -y git build-essential

# Orbbec SDK dependencies
sudo apt-get install -y libusb-1.0-0-dev
```

#### 2. Setup project
```bash
mkdir -p /opt/ur5_control
cd /opt/ur5_control
git clone <repo> .
```

#### 3. Virtual environment
```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

#### 4. Orbbec SDK (Linux)
```bash
# Download từ https://www.orbbec.com/
# Hoặc build từ source
cd /tmp
wget https://github.com/orbbec/OrbbecSDK/releases/download/v1.x.x/OrbbecSDK_Linux.tar.gz
tar xzf OrbbecSDK_Linux.tar.gz
cd OrbbecSDK
sudo apt-get install -y ./orbbec*.deb

# Test
python3 -c "import ob; print('OK')"
```

#### 5. Create .env
```bash
cp .env.example .env
nano .env  # Edit với cấu hình thực tế
```

#### 6. Create systemd service
```bash
sudo nano /etc/systemd/system/ur5-pc2.service
```

**File nội dung:**
```ini
[Unit]
Description=UR5 Robot Control Server (PC2)
After=network.target

[Service]
Type=simple
User=ur5_user
WorkingDirectory=/opt/ur5_control
Environment="PATH=/opt/ur5_control/venv/bin"
ExecStart=/opt/ur5_control/venv/bin/python /opt/ur5_control/app.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

#### 7. Enable và start service
```bash
sudo systemctl enable ur5-pc2
sudo systemctl start ur5-pc2

# Check status
sudo systemctl status ur5-pc2

# View logs
sudo journalctl -u ur5-pc2 -f
```

### Windows

#### 1. Python setup
- Download Python 3.8+ từ python.org
- Cài đặt, check "Add to PATH"

#### 2. Clone repo
```cmd
cd C:\opt
git clone <repo> ur5_control
cd ur5_control
```

#### 3. Virtual environment
```cmd
python -m venv venv
venv\Scripts\activate
pip install --upgrade pip
pip install -r requirements.txt
```

#### 4. Orbbec SDK
- Download installer từ Orbbec website
- Cài đặt
- Verify: `python -c "import ob"`

#### 5. .env file
```cmd
copy .env.example .env
notepad .env
```

#### 6. Run as service (NSSM hoặc Task Scheduler)

**Option A: NSSM (Recommended)**
```cmd
# Download nssm từ https://nssm.cc/
nssm install UR5-PC2 C:\opt\ur5_control\venv\Scripts\python.exe c:\opt\ur5_control\app.py
nssm start UR5-PC2
```

**Option B: Task Scheduler**
- Create scheduled task để run `app.py` at startup
- Set user with camera permissions

---

## Option 2: Docker Deployment

### Dockerfile
```dockerfile
FROM python:3.9-slim-bullseye

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    libusb-1.0-0-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Orbbec SDK
RUN cd /tmp && \
    wget https://github.com/orbbec/OrbbecSDK/releases/download/v1.x.x/OrbbecSDK_Linux.tar.gz && \
    tar xzf OrbbecSDK_Linux.tar.gz && \
    cd OrbbecSDK && \
    apt-get install -y ./orbbec*.deb && \
    rm -rf /tmp/OrbbecSDK*

# Copy code
COPY . /app

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Create logs directory
RUN mkdir -p /app/logs

# Expose port
EXPOSE 5001

# Run app
CMD ["python", "app.py"]
```

### docker-compose.yml
```yaml
version: '3.8'

services:
  ur5-pc2:
    build: .
    container_name: ur5-pc2
    ports:
      - "5001:5001"
    environment:
      ROBOT_IP: 192.168.125.11
      PC2_HOST: 0.0.0.0
      PC2_PORT: 5001
      PC1_BASE_URL: http://192.168.1.100:5000
      PC1_CALLBACK_ENABLED: "False"
      LOG_LEVEL: INFO
    volumes:
      - ./logs:/app/logs
      - /dev/bus/usb:/dev/bus/usb  # Pass USB untuk camera
    privileged: true  # Cần cho USB access
    restart: always
    networks:
      - ur5_network

networks:
  ur5_network:
    driver: bridge
```

### Build & Run
```bash
# Build
docker build -t ur5-pc2:latest .

# Run with docker-compose
docker-compose up -d

# Check logs
docker-compose logs -f

# Stop
docker-compose down
```

---

## Gunicorn (Production WSGI)

### Install
```bash
pip install gunicorn
```

### Create gunicorn_config.py
```python
# gunicorn_config.py
import multiprocessing

bind = "0.0.0.0:5001"
workers = 1  # Single worker để tránh race condition với robot
threads = 1
worker_class = "sync"
timeout = 120
keepalive = 5
loglevel = "info"
```

### Run
```bash
gunicorn -c gunicorn_config.py app:app
```

### Systemd service (gunicorn version)
```ini
[Service]
ExecStart=/opt/ur5_control/venv/bin/gunicorn -c /opt/ur5_control/gunicorn_config.py app:app
```

---

## Nginx Reverse Proxy (Optional)

Dùng Nginx để reverse proxy và SSL termination.

### Install
```bash
sudo apt-get install -y nginx
```

### Config: /etc/nginx/sites-available/ur5-pc2
```nginx
upstream ur5_backend {
    server 127.0.0.1:5001;
}

server {
    listen 80;
    server_name ur5-pc2.example.com;

    location / {
        proxy_pass http://ur5_backend;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        proxy_redirect off;
    }
}
```

### Enable
```bash
sudo ln -s /etc/nginx/sites-available/ur5-pc2 /etc/nginx/sites-enabled/
sudo systemctl restart nginx
```

---

## Monitoring & Logging

### Log rotation
Tạo `/etc/logrotate.d/ur5-pc2`:
```
/opt/ur5_control/logs/*.log {
    daily
    rotate 7
    compress
    delaycompress
    notifempty
    create 0640 ur5_user ur5_user
    sharedscripts
}
```

### Health check
```bash
curl http://localhost:5001/api/ur5/health
```

### Monitoring stack (Prometheus + Grafana)
```bash
# Add Prometheus client
pip install prometheus-flask-exporter
```

**app.py**:
```python
from prometheus_flask_exporter import PrometheusMetrics

app = create_app()
metrics = PrometheusMetrics(app)
```

Access metrics ở `/metrics`.

---

## Network Security

### Firewall rules
```bash
# Allow only PC1 to access PC2
sudo ufw allow from 192.168.1.100 to any port 5001

# Allow robot network
sudo ufw allow from 192.168.125.0/24 to any port 29999
sudo ufw allow from 192.168.125.0/24 to any port 30002
sudo ufw allow from 192.168.125.0/24 to any port 30004
```

### API authentication (Optional)
Add header token check:
```python
from functools import wraps

def require_auth(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        token = request.headers.get('X-API-Key')
        if token != config.API_KEY:
            return {"error": "Unauthorized"}, 401
        return f(*args, **kwargs)
    return decorated

@ur5_bp.route("/execute", methods=["POST"])
@require_auth
def execute_job():
    ...
```

---

## CI/CD Pipeline (GitHub Actions)

### .github/workflows/deploy.yml
```yaml
name: Deploy to PC2

on:
  push:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - uses: actions/setup-python@v2
        with:
          python-version: 3.9
      - run: pip install -r requirements.txt
      - run: python -m pytest tests/

  deploy:
    needs: test
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Deploy to PC2
        run: |
          ssh -i ${{ secrets.SSH_KEY }} ur5@pc2.example.com << 'EOF'
            cd /opt/ur5_control
            git pull origin main
            source venv/bin/activate
            pip install -r requirements.txt
            systemctl restart ur5-pc2
          EOF
```

---

## Troubleshooting

### Port already in use
```bash
sudo lsof -i :5001
sudo kill -9 <PID>
```

### Camera not detected in Docker
- Kiểm tra USB pass-through: `docker exec ur5-pc2 lsusb`
- Check udev rules: `sudo usermod -aG plugdev <user>`

### RTDE connection timeout
- Kiểm tra firewall port 30004
- Robot có bật RTDE interface?

### Memory leak
- Monitor RAM: `docker stats ur5-pc2`
- Job store cleanup: `JobStore.cleanup_old_jobs()`

---

## Backup & Recovery

### Backup logs
```bash
tar czf ur5_logs_$(date +%Y%m%d).tar.gz /opt/ur5_control/logs/
```

### Config backup
```bash
cp /opt/ur5_control/.env /backup/.env.backup
```

---

## Performance tuning

### Python optimization
```bash
# Use PyPy for faster execution (if compatible)
pip install pypy3
```

### UR5 motion optimization
- Tăng `JOINT_VEL` cho motion nhanh
- Tăng `LINEAR_VEL` cho pick/place nhanh
- Trade-off: tốc độ vs accuracy

### Camera optimization
- Reduce resolution: 1280x720 → 640x480
- Reduce framerate: 30 fps → 15 fps
- Enable depth hole fill

---

## References

- [Systemd Service Man](https://www.freedesktop.org/software/systemd/man/systemd.service.html)
- [Docker Documentation](https://docs.docker.com/)
- [Gunicorn Documentation](https://docs.gunicorn.org/)
- [Nginx Configuration](https://nginx.org/en/docs/)

