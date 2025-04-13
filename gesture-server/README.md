# Gesture Authentication Server

A simple FastAPI server for gesture-based authentication.

## Setup

1. Install dependencies:
```
make install
```
or
```
poetry install
```

2. Run the server:
```
make dev
```

Alternatively, you can run the server directly with:
```
poetry run python run_server.py
```

Or with the full uvicorn command:
```
poetry run python -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8300
```

3. Open your browser to http://localhost:8300 