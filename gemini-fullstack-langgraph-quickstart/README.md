## 导出和导入 Docker Compose 项目

### 导出项目为 tar.gz

在源机器上执行以下步骤：

```bash
docker build -t gemini-fullstack-langgraph -f Dockerfile .
```

```powershell
# 保存镜像
docker save gemini-fullstack-langgraph -o gemini-fullstack-langgraph.tar
docker save docker.io/redis:6 -o redis-6.tar
docker save docker.io/postgres:16 -o postgres-16.tar
```

### 在目标机器上导入项目

**1. 上传项目文件：**

```bash
docker-compose.yml 
Dockerfile 
Makefile
*.tar // 所有的压缩文件
```

**2. 加载 Docker 镜像：**

```bash
docker load -i gemini-fullstack-langgraph.tar
docker load -i redis-6.tar
docker load -i postgres-16.tar
```

**4. 设置环境变量并启动服务：**

Linux
```bash
LANGSMITH_API_KEY="lsv2****" docker-compose up -d
```

`http://localhost:8123/app/`
API at `http://localhost:8123`.
