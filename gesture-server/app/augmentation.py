import numpy as np
import pandas as pd
from scipy import signal, interpolate
from typing import List, Dict, Tuple, Any, Optional, Union
import random
import logging
import io

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("gesture_augmentation")

class IMUGestureAugmenter:
    """
    Class for generating synthetic IMU gesture data based on original samples.
    Specifically designed for accelerometer and gyroscope data.
    """
    
    # Constants that can be tuned based on your specific IMU characteristics
    ACCEL_COLUMNS = ['acc_x', 'acc_y', 'acc_z']
    GYRO_COLUMNS = ['gyro_x', 'gyro_y', 'gyro_z']
    TIMESTAMP_COLUMN = 'timestamp'
    
    def __init__(self, random_seed: Optional[int] = None):
        """
        Initialize the augmenter with optional random seed for reproducibility.
        
        Args:
            random_seed: Optional seed for random number generation
        """
        if random_seed is not None:
            np.random.seed(random_seed)
            random.seed(random_seed)
        
        # Track metrics
        self.augmentation_stats = {
            "originals": 0,
            "generated": 0,
            "errors": 0
        }
    
    def parse_csv_data(self, csv_data: str) -> pd.DataFrame:
        """
        Parse CSV string into a pandas DataFrame.
        
        Args:
            csv_data: String containing CSV data
            
        Returns:
            pandas DataFrame with the parsed data
        """
        try:
            df = pd.read_csv(io.StringIO(csv_data))
            return df
        except Exception as e:
            logger.error(f"Error parsing CSV data: {e}")
            raise ValueError(f"Could not parse CSV data: {e}")
    
    def add_noise(self, df: pd.DataFrame, 
                 accel_noise_std: float = 0.02, 
                 gyro_noise_std: float = 0.5) -> pd.DataFrame:
        """
        Add Gaussian noise to IMU signals.
        
        Args:
            df: Input DataFrame containing IMU data
            accel_noise_std: Standard deviation of noise for accelerometer (g)
            gyro_noise_std: Standard deviation of noise for gyroscope (deg/s)
            
        Returns:
            DataFrame with noise added
        """
        df_noisy = df.copy()
        
        # Add noise to accelerometer data
        for col in self.ACCEL_COLUMNS:
            if col in df_noisy.columns:
                noise = np.random.normal(0, accel_noise_std, size=len(df_noisy))
                df_noisy[col] = df_noisy[col] + noise
        
        # Add noise to gyroscope data
        for col in self.GYRO_COLUMNS:
            if col in df_noisy.columns:
                noise = np.random.normal(0, gyro_noise_std, size=len(df_noisy))
                df_noisy[col] = df_noisy[col] + noise
                
        return df_noisy
    
    def scale_magnitude(self, df: pd.DataFrame, 
                       accel_scale: float = 1.0, 
                       gyro_scale: float = 1.0) -> pd.DataFrame:
        """
        Scale the magnitude of IMU signals.
        
        Args:
            df: Input DataFrame containing IMU data
            accel_scale: Scaling factor for accelerometer
            gyro_scale: Scaling factor for gyroscope
            
        Returns:
            DataFrame with scaled signals
        """
        df_scaled = df.copy()
        
        # Scale accelerometer data
        for col in self.ACCEL_COLUMNS:
            if col in df_scaled.columns:
                df_scaled[col] = df_scaled[col] * accel_scale
        
        # Scale gyroscope data
        for col in self.GYRO_COLUMNS:
            if col in df_scaled.columns:
                df_scaled[col] = df_scaled[col] * gyro_scale
                
        return df_scaled
    
    def time_warp(self, df: pd.DataFrame, 
                 warp_factor: float = 0.2, 
                 segments: int = 5) -> pd.DataFrame:
        """
        Apply time warping to IMU signals by stretching or compressing different segments.
        
        Args:
            df: Input DataFrame containing IMU data
            warp_factor: Maximum warping factor (0.2 means ±20% time scaling)
            segments: Number of segments to split the signal into for warping
            
        Returns:
            DataFrame with time-warped signals
        """
        df_warped = df.copy()
        n_samples = len(df)
        
        if n_samples <= segments:
            return df_warped  # Can't warp if too few samples
        
        # Determine original timestamps or create synthetic ones
        if self.TIMESTAMP_COLUMN in df_warped.columns:
            orig_times = df_warped[self.TIMESTAMP_COLUMN].values
        else:
            orig_times = np.arange(n_samples)
        
        # Create segment boundaries
        segment_bounds = np.linspace(0, n_samples-1, segments+1).astype(int)
        
        # Generate warping factors for each segment
        warp_factors = 1.0 + np.random.uniform(-warp_factor, warp_factor, size=segments)
        
        # Build new timeline
        new_times = np.zeros(n_samples)
        cumulative_time = 0
        
        for i in range(segments):
            start, end = segment_bounds[i], segment_bounds[i+1]
            segment_duration = orig_times[end] - orig_times[start]
            warped_duration = segment_duration * warp_factors[i]
            
            if i < segments - 1:  # All but the last segment
                for j in range(start, end):
                    progress_ratio = (j - start) / (end - start)
                    new_times[j] = orig_times[start] + progress_ratio * warped_duration
            else:  # Handle the last segment separately to ensure it ends precisely at the original end time
                for j in range(start, end):
                    progress_ratio = (j - start) / (end - start)
                    new_times[j] = orig_times[start] + progress_ratio * segment_duration
        
        # Interpolate all columns to the new timeline
        result_data = {}
        
        # Keep timestamp column unchanged if it exists
        if self.TIMESTAMP_COLUMN in df_warped.columns:
            result_data[self.TIMESTAMP_COLUMN] = df_warped[self.TIMESTAMP_COLUMN].copy()
        
        # Interpolate sensor columns
        for col in df_warped.columns:
            if col != self.TIMESTAMP_COLUMN:
                try:
                    f = interpolate.interp1d(new_times, df_warped[col].values, 
                                            kind='linear', fill_value='extrapolate')
                    result_data[col] = f(orig_times)
                except Exception as e:
                    logger.warning(f"Could not warp column {col}: {e}")
                    result_data[col] = df_warped[col].copy()
        
        return pd.DataFrame(result_data)
    
    def rotate_signals(self, df: pd.DataFrame, max_angle_deg: float = 5.0) -> pd.DataFrame:
        """
        Apply a small 3D rotation to the signals to simulate different orientations.
        
        Args:
            df: Input DataFrame containing IMU data
            max_angle_deg: Maximum rotation angle in degrees
            
        Returns:
            DataFrame with rotated IMU signals
        """
        df_rotated = df.copy()
        
        # Check if we have all required columns
        accel_exists = all(col in df.columns for col in self.ACCEL_COLUMNS)
        gyro_exists = all(col in df.columns for col in self.GYRO_COLUMNS)
        
        if not (accel_exists and gyro_exists):
            logger.warning("Cannot rotate signals: missing required columns")
            return df_rotated
            
        # Generate random rotation angles in radians
        angle_x = np.radians(np.random.uniform(-max_angle_deg, max_angle_deg))
        angle_y = np.radians(np.random.uniform(-max_angle_deg, max_angle_deg))
        angle_z = np.radians(np.random.uniform(-max_angle_deg, max_angle_deg))
        
        # Rotation matrices
        def rotation_matrix_x(angle):
            return np.array([
                [1, 0, 0],
                [0, np.cos(angle), -np.sin(angle)],
                [0, np.sin(angle), np.cos(angle)]
            ])
        
        def rotation_matrix_y(angle):
            return np.array([
                [np.cos(angle), 0, np.sin(angle)],
                [0, 1, 0],
                [-np.sin(angle), 0, np.cos(angle)]
            ])
        
        def rotation_matrix_z(angle):
            return np.array([
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1]
            ])
        
        # Combined rotation matrix
        R = rotation_matrix_z(angle_z) @ rotation_matrix_y(angle_y) @ rotation_matrix_x(angle_x)
        
        # Apply rotation to each sample
        accel_data = df[self.ACCEL_COLUMNS].values
        gyro_data = df[self.GYRO_COLUMNS].values
        
        rotated_accel = np.zeros_like(accel_data)
        rotated_gyro = np.zeros_like(gyro_data)
        
        for i in range(len(df)):
            rotated_accel[i] = R @ accel_data[i]
            rotated_gyro[i] = R @ gyro_data[i]
        
        # Update the DataFrame
        for i, col in enumerate(self.ACCEL_COLUMNS):
            df_rotated[col] = rotated_accel[:, i]
            
        for i, col in enumerate(self.GYRO_COLUMNS):
            df_rotated[col] = rotated_gyro[:, i]
            
        return df_rotated
    
    def selective_filtering(self, df: pd.DataFrame, 
                           filter_strength: float = 0.3) -> pd.DataFrame:
        """
        Apply selective filtering to the signals to simulate different movement qualities.
        
        Args:
            df: Input DataFrame containing IMU data
            filter_strength: Strength of the filter (0-1)
            
        Returns:
            DataFrame with selectively filtered signals
        """
        df_filtered = df.copy()
        
        # Determine filter order based on the length of data
        n = len(df)
        order = max(min(int(n / 10), 5), 2)  # Adaptive order between 2 and 5
        
        # Calculate cutoff frequency (normalized between 0 and 1)
        cutoff = 0.5 - 0.4 * filter_strength  # Map filter_strength to frequency
        
        # Design Butterworth filter
        b, a = signal.butter(order, cutoff, 'low')
        
        # Apply filtering to sensor columns
        all_columns = self.ACCEL_COLUMNS + self.GYRO_COLUMNS
        for col in all_columns:
            if col in df.columns:
                try:
                    # Apply filtfilt for zero-phase filtering (no time delay)
                    df_filtered[col] = signal.filtfilt(b, a, df[col])
                except Exception as e:
                    logger.warning(f"Could not filter column {col}: {e}")
        
        return df_filtered
    
    def combine_augmentations(self, df: pd.DataFrame, 
                             methods: List[str], 
                             params: Optional[Dict] = None) -> pd.DataFrame:
        """
        Apply multiple augmentation methods in sequence.
        
        Args:
            df: Input DataFrame containing IMU data
            methods: List of augmentation method names to apply
            params: Optional dictionary of parameters for each method
            
        Returns:
            DataFrame with combined augmentations
        """
        df_result = df.copy()
        
        # Define default parameters
        default_params = {
            'add_noise': {'accel_noise_std': 0.02, 'gyro_noise_std': 0.5},
            'scale_magnitude': {'accel_scale': random.uniform(0.85, 1.15), 
                               'gyro_scale': random.uniform(0.85, 1.15)},
            'time_warp': {'warp_factor': 0.2, 'segments': 5},
            'rotate_signals': {'max_angle_deg': 5.0},
            'selective_filtering': {'filter_strength': random.uniform(0.1, 0.4)}
        }
        
        # Use provided parameters or defaults
        if params is None:
            params = {}
            
        # Apply methods in sequence
        for method in methods:
            if method == 'add_noise':
                method_params = {**default_params['add_noise'], **params.get('add_noise', {})}
                df_result = self.add_noise(df_result, **method_params)
            
            elif method == 'scale_magnitude':
                method_params = {**default_params['scale_magnitude'], **params.get('scale_magnitude', {})}
                df_result = self.scale_magnitude(df_result, **method_params)
            
            elif method == 'time_warp':
                method_params = {**default_params['time_warp'], **params.get('time_warp', {})}
                df_result = self.time_warp(df_result, **method_params)
            
            elif method == 'rotate_signals':
                method_params = {**default_params['rotate_signals'], **params.get('rotate_signals', {})}
                df_result = self.rotate_signals(df_result, **method_params)
            
            elif method == 'selective_filtering':
                method_params = {**default_params['selective_filtering'], 
                                **params.get('selective_filtering', {})}
                df_result = self.selective_filtering(df_result, **method_params)
            
            else:
                logger.warning(f"Unknown augmentation method: {method}")
        
        return df_result
    
    def generate_variations(self, csv_data_list: List[str], 
                          variations_per_original: int = 5) -> List[str]:
        """
        Generate multiple variations from each original CSV dataset.
        
        Args:
            csv_data_list: List of CSV strings containing original IMU data
            variations_per_original: Number of variations to generate per original
            
        Returns:
            List of CSV strings containing the original and augmented data
        """
        results = []
        
        # Track stats
        self.augmentation_stats["originals"] = len(csv_data_list)
        
        # Define augmentation recipes for variety
        augmentation_recipes = [
            # Single techniques
            ['add_noise'],
            ['time_warp'],
            ['scale_magnitude'],
            ['rotate_signals'],
            ['selective_filtering'],
            
            # Combinations
            ['add_noise', 'time_warp'],
            ['scale_magnitude', 'add_noise'],
            ['time_warp', 'selective_filtering'],
            ['rotate_signals', 'scale_magnitude'],
            ['add_noise', 'rotate_signals'],
            
            # Triple combinations
            ['add_noise', 'time_warp', 'selective_filtering'],
            ['rotate_signals', 'scale_magnitude', 'time_warp'],
            ['scale_magnitude', 'add_noise', 'selective_filtering'],
            ['time_warp', 'rotate_signals', 'add_noise'],
            ['selective_filtering', 'scale_magnitude', 'rotate_signals']
        ]
        
        # Keep the original data
        results.extend(csv_data_list)
        
        # Generate variations
        for i, csv_data in enumerate(csv_data_list):
            try:
                # Parse original data
                df_original = self.parse_csv_data(csv_data)
                
                # Generate variations
                for v in range(variations_per_original):
                    # Select recipe (cycling through them)
                    recipe_idx = (i * variations_per_original + v) % len(augmentation_recipes)
                    recipe = augmentation_recipes[recipe_idx]
                    
                    # Apply augmentations
                    df_augmented = self.combine_augmentations(df_original, recipe)
                    
                    # Convert back to CSV
                    csv_augmented = df_augmented.to_csv(index=False)
                    results.append(csv_augmented)
                    
                    # Update stats
                    self.augmentation_stats["generated"] += 1
                    
            except Exception as e:
                logger.error(f"Error generating variations for dataset {i+1}: {e}")
                self.augmentation_stats["errors"] += 1
        
        return results

def augment_gesture_data(csv_data_list: List[str], 
                        variations_per_original: int = 5, 
                        random_seed: Optional[int] = None) -> List[str]:
    """
    Convenience function to augment gesture data without directly using the class.
    
    Args:
        csv_data_list: List of CSV strings containing original IMU data
        variations_per_original: Number of variations to generate per original
        random_seed: Optional seed for random number generation
        
    Returns:
        List of CSV strings containing the original and augmented data
    """
    augmenter = IMUGestureAugmenter(random_seed=random_seed)
    return augmenter.generate_variations(csv_data_list, variations_per_original) 