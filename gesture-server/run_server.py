#!/usr/bin/env python

import uvicorn

def main():
    uvicorn.run("app.main:app", host="0.0.0.0", port=8300, reload=True)

if __name__ == "__main__":
    main() 