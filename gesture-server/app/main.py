from fastapi import FastAPI, Request, Form, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from typing import List, Dict, Any, Optional
from pydantic import BaseModel, Field
import os
import json
import numpy as np
import io
import csv
import re

# Import our augmentation module
from app.augmentation import augment_gesture_data, IMUGestureAugmenter

app = FastAPI()

# Set up templates
templates_dir = os.path.join(os.path.dirname(__file__), "templates")
templates = Jinja2Templates(directory=templates_dir)

# Set up static files
static_dir = os.path.join(os.path.dirname(__file__), "static")
app.mount("/static", StaticFiles(directory=static_dir), name="static")

# Define models for the API
class AugmentationRequest(BaseModel):
    gesture_name: str
    csv_data: List[str] = Field(..., min_items=1, max_items=10)
    variations_per_original: int = Field(5, ge=1, le=20)

def detect_and_add_headers(csv_text):
    """Check if headers are present, add them if missing"""
    # Check if the CSV already has headers that match our expected pattern
    lines = csv_text.strip().split('\n')
    if not lines:
        return csv_text
    
    first_line = lines[0].strip()
    
    # Check if the first line has headers (headers should contain text, not just numbers)
    # Headers typically contain alpha characters, while data is numeric
    has_headers = False
    
    # If first line matches expected headers (case insensitive)
    if re.match(r'timestamp.*acc.*gyro', first_line.lower(), re.IGNORECASE):
        has_headers = True
    # Check if first line contains mostly non-numeric values
    elif sum(1 for item in first_line.split(',') if not re.match(r'^-?\d+\.?\d*$', item.strip())) > 3:
        has_headers = True
    
    if has_headers:
        return csv_text
    else:
        # Add headers
        headers_line = 'timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z'
        return headers_line + '\n' + csv_text

@app.get("/")
def read_root():
    return {"message": "Hello World"} 

@app.get("/combined", response_class=HTMLResponse)
async def combined_form(request: Request):
    """
    Render the combined form page
    """
    return templates.TemplateResponse("combined_form.html", {"request": request}) 

@app.post("/gestures/augment")
async def augment_gestures(request: AugmentationRequest):
    """
    Augment gesture data to create synthetic variations.
    
    This endpoint takes one or more CSV datasets and generates synthetic variations
    using various augmentation techniques like noise addition, time warping, 
    magnitude scaling, rotation, and selective filtering.
    
    Args:
        request: AugmentationRequest containing gesture name, CSV data, and configuration
        
    Returns:
        Redirects to the visualization page with the augmented data
    """
    try:
        # Process and ensure headers exist in each CSV dataset
        processed_csv_list = [detect_and_add_headers(csv_data) for csv_data in request.csv_data]
        
        # Generate augmented datasets
        augmented_data = augment_gesture_data(
            processed_csv_list, 
            variations_per_original=request.variations_per_original
        )
        
        # Prepare response
        datasets = []
        
        # Process each dataset for the response
        for i, csv_text in enumerate(augmented_data):
            # Parse CSV content to extract basic info
            csv_reader = csv.reader(io.StringIO(csv_text))
            
            # Extract headers
            headers = next(csv_reader, None)
            
            # Count rows
            row_count = sum(1 for _ in csv_reader) + 1  # +1 for header
            
            # Determine if this is an original or generated dataset
            is_original = i < len(request.csv_data)
            
            dataset_info = {
                "dataset_id": i + 1,
                "is_original": is_original,
                "type": "original" if is_original else "augmented",
                "row_count": row_count,
                "csv_data": csv_text
            }
            
            datasets.append(dataset_info)
        
        response_data = {
            "gesture_name": request.gesture_name,
            "original_count": len(request.csv_data),
            "generated_count": len(augmented_data) - len(request.csv_data),
            "total_count": len(augmented_data),
            "datasets": datasets
        }
        
        # Store the response data in a JSON file to be accessed by the visualization page
        os.makedirs(os.path.join(static_dir, "data"), exist_ok=True)
        session_id = np.random.randint(10000000, 99999999)
        json_path = os.path.join(static_dir, "data", f"augmentation_{session_id}.json")
        
        with open(json_path, "w") as f:
            json.dump(response_data, f)
        
        # Redirect to the visualization page with the session ID
        return RedirectResponse(url=f"/visualize/{session_id}", status_code=303)
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error augmenting gestures: {str(e)}")

@app.get("/visualize/{session_id}", response_class=HTMLResponse)
async def visualize_augmented_data(request: Request, session_id: str):
    """
    Render a visualization page for the augmented data.
    
    Args:
        request: The request object
        session_id: The session ID for the augmentation data
        
    Returns:
        HTML visualization page
    """
    try:
        # Load the augmentation data
        json_path = os.path.join(static_dir, "data", f"augmentation_{session_id}.json")
        
        if not os.path.exists(json_path):
            raise HTTPException(status_code=404, detail="Augmentation data not found")
        
        with open(json_path, "r") as f:
            augmentation_data = json.load(f)
        
        # Pass the data to the template
        return templates.TemplateResponse(
            "visualization.html", 
            {
                "request": request, 
                "session_id": session_id,
                "gesture_name": augmentation_data["gesture_name"],
                "original_count": augmentation_data["original_count"],
                "generated_count": augmentation_data["generated_count"],
                "total_count": augmentation_data["total_count"]
            }
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error rendering visualization: {str(e)}")

@app.get("/api/augmentation-data/{session_id}")
async def get_augmentation_data(session_id: str):
    """
    API endpoint to get the augmentation data for a specific session.
    
    Args:
        session_id: The session ID for the augmentation data
        
    Returns:
        JSON data with the augmentation results
    """
    try:
        json_path = os.path.join(static_dir, "data", f"augmentation_{session_id}.json")
        
        if not os.path.exists(json_path):
            raise HTTPException(status_code=404, detail="Augmentation data not found")
        
        with open(json_path, "r") as f:
            augmentation_data = json.load(f)
        
        return augmentation_data
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting augmentation data: {str(e)}") 