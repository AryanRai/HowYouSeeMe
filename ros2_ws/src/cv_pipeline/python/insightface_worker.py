#!/usr/bin/env python3
"""
InsightFace Worker - Face Detection, Recognition, and Liveness Detection
Supports modular modes for pipeline composition
"""

import numpy as np
import cv2
import time
import os
import pickle
import json
from pathlib import Path
from typing import Dict, List, Tuple, Optional

try:
    import insightface
    from insightface.app import FaceAnalysis
    INSIGHTFACE_AVAILABLE = True
except ImportError:
    INSIGHTFACE_AVAILABLE = False
    print("Warning: InsightFace not installed. Install with: pip install insightface onnxruntime-gpu")


class InsightFaceWorker:
    """
    InsightFace worker for face detection, recognition, and liveness detection
    
    Modes:
    - detect: Face detection only (returns bboxes and landmarks)
    - recognize: Face recognition only (assumes cropped face input)
    - detect_recognize: Full pipeline (detect + recognize)
    - register: Register new face to database
    - liveness: Liveness detection using depth
    - analyze: Full analysis (detect + recognize + liveness + attributes)
    """
    
    def __init__(self, device="cuda"):
        if not INSIGHTFACE_AVAILABLE:
            raise ImportError("InsightFace not installed")
        
        self.device = device
        self.app = None
        self.detector = None
        self.recognizer = None
        
        # Face database
        self.database_path = Path("data/faces")
        self.database_path.mkdir(parents=True, exist_ok=True)
        self.db_file = self.database_path / "face_database.pkl"
        self.metadata_file = self.database_path / "metadata.json"
        
        self.face_database = self.load_database()
        self.metadata = self.load_metadata()
        
        # Recognition parameters
        self.similarity_threshold = 0.6
        self.min_face_size = 20
        
        print("InsightFace Worker initialized")
    
    def load_models(self, model_pack="buffalo_l"):
        """Load InsightFace models"""
        print(f"Loading InsightFace models: {model_pack}")
        
        # Full analysis app (includes detection, recognition, age/gender)
        self.app = FaceAnalysis(name=model_pack, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
        
        # Detection only app
        self.detector = FaceAnalysis(
            name=model_pack,
            allowed_modules=['detection'],
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )
        
        # Note: We use self.app for recognition since InsightFace requires detection module
        # The recognizer needs detection to work properly
        self.recognizer = self.app  # Use full app for recognition
        
        print("InsightFace models loaded successfully")
    
    def prepare(self, det_size=(640, 640), det_thresh=0.5):
        """Prepare models for inference"""
        ctx_id = 0 if self.device == "cuda" else -1
        
        if self.app:
            self.app.prepare(ctx_id=ctx_id, det_size=det_size, det_thresh=det_thresh)
        if self.detector:
            self.detector.prepare(ctx_id=ctx_id, det_size=det_size, det_thresh=det_thresh)
        if self.recognizer:
            self.recognizer.prepare(ctx_id=ctx_id)
        
        print(f"Models prepared: det_size={det_size}, det_thresh={det_thresh}")
    
    def load_database(self) -> Dict:
        """Load face database from disk"""
        if self.db_file.exists():
            with open(self.db_file, 'rb') as f:
                return pickle.load(f)
        return {}
    
    def save_database(self):
        """Save face database to disk"""
        with open(self.db_file, 'wb') as f:
            pickle.dump(self.face_database, f)
    
    def load_metadata(self) -> Dict:
        """Load metadata from disk"""
        if self.metadata_file.exists():
            with open(self.metadata_file, 'r') as f:
                return json.load(f)
        return {}
    
    def save_metadata(self):
        """Save metadata to disk"""
        with open(self.metadata_file, 'w') as f:
            json.dump(self.metadata, f, indent=2)
    
    def detect_faces(self, image: np.ndarray, params: Dict) -> Dict:
        """
        Mode: detect
        Detect faces only, return bounding boxes and landmarks
        """
        start_time = time.time()
        
        det_size = int(params.get('det_size', 640))
        max_num = int(params.get('max_num', 0))  # 0 = no limit
        
        # Detect faces
        faces = self.detector.get(image, max_num=max_num)
        
        # Extract results
        results = []
        for face in faces:
            results.append({
                'bbox': face.bbox.tolist(),
                'det_score': float(face.det_score),
                'landmarks': face.kps.tolist() if face.kps is not None else None
            })
        
        processing_time = time.time() - start_time
        
        return {
            'mode': 'detect',
            'num_faces': len(results),
            'faces': results,
            'processing_time': processing_time
        }
    
    def recognize_face(self, face_image: np.ndarray, params: Dict) -> Dict:
        """
        Mode: recognize
        Recognize a single face (assumes already cropped/aligned)
        """
        start_time = time.time()
        
        threshold = float(params.get('threshold', self.similarity_threshold))
        
        # Get embedding
        faces = self.app.get(face_image)
        
        if len(faces) == 0:
            return {
                'mode': 'recognize',
                'recognized': False,
                'reason': 'no_face_detected',
                'processing_time': time.time() - start_time
            }
        
        face = faces[0]
        embedding = face.normed_embedding
        
        # Match against database
        best_match = self.match_face(embedding, threshold)
        
        processing_time = time.time() - start_time
        
        if best_match:
            return {
                'mode': 'recognize',
                'recognized': True,
                'person_id': best_match['person_id'],
                'name': best_match['name'],
                'similarity': best_match['similarity'],
                'confidence': best_match['similarity'],
                'processing_time': processing_time
            }
        else:
            return {
                'mode': 'recognize',
                'recognized': False,
                'reason': 'no_match_found',
                'processing_time': processing_time
            }
    
    def detect_and_recognize(self, image: np.ndarray, params: Dict) -> Dict:
        """
        Mode: detect_recognize
        Full pipeline: detect faces then recognize each one
        """
        start_time = time.time()
        
        threshold = float(params.get('threshold', self.similarity_threshold))
        max_num = int(params.get('max_num', 0))
        
        # Detect and analyze faces
        faces = self.app.get(image, max_num=max_num)
        
        results = []
        for face in faces:
            embedding = face.normed_embedding
            best_match = self.match_face(embedding, threshold)
            
            face_result = {
                'bbox': face.bbox.tolist(),
                'det_score': float(face.det_score),
                'landmarks': face.kps.tolist() if face.kps is not None else None,
            }
            
            if best_match:
                face_result.update({
                    'recognized': True,
                    'person_id': best_match['person_id'],
                    'name': best_match['name'],
                    'similarity': best_match['similarity']
                })
            else:
                face_result.update({
                    'recognized': False,
                    'person_id': 'unknown'
                })
            
            # Add attributes if available
            if hasattr(face, 'age') and face.age is not None:
                face_result['age'] = int(face.age)
            if hasattr(face, 'gender') and face.gender is not None:
                face_result['gender'] = 'M' if face.gender == 1 else 'F'
            
            results.append(face_result)
        
        processing_time = time.time() - start_time
        
        return {
            'mode': 'detect_recognize',
            'num_faces': len(results),
            'faces': results,
            'processing_time': processing_time
        }
    
    def register_face(self, image: np.ndarray, params: Dict) -> Dict:
        """
        Mode: register
        Register a new face to the database
        """
        start_time = time.time()
        
        name = params.get('name', 'unknown')
        person_id = params.get('person_id', f"person_{len(self.face_database):03d}")
        
        # Detect face
        faces = self.app.get(image)
        
        if len(faces) == 0:
            return {
                'mode': 'register',
                'success': False,
                'reason': 'no_face_detected',
                'processing_time': time.time() - start_time
            }
        
        if len(faces) > 1:
            return {
                'mode': 'register',
                'success': False,
                'reason': 'multiple_faces_detected',
                'num_faces': len(faces),
                'processing_time': time.time() - start_time
            }
        
        face = faces[0]
        embedding = face.normed_embedding
        
        # Add to database
        if person_id not in self.face_database:
            self.face_database[person_id] = {
                'embeddings': [],
                'name': name
            }
            self.metadata[person_id] = {
                'name': name,
                'first_seen': time.strftime('%Y-%m-%d %H:%M:%S'),
                'encounter_count': 0,
                'samples': 0
            }
        
        self.face_database[person_id]['embeddings'].append(embedding)
        self.metadata[person_id]['samples'] = len(self.face_database[person_id]['embeddings'])
        self.metadata[person_id]['last_updated'] = time.strftime('%Y-%m-%d %H:%M:%S')
        
        # Save to disk
        self.save_database()
        self.save_metadata()
        
        processing_time = time.time() - start_time
        
        return {
            'mode': 'register',
            'success': True,
            'person_id': person_id,
            'name': name,
            'num_samples': len(self.face_database[person_id]['embeddings']),
            'processing_time': processing_time
        }
    
    def check_liveness(self, image: np.ndarray, depth_image: Optional[np.ndarray], params: Dict) -> Dict:
        """
        Mode: liveness
        Check if face is live using depth information
        """
        start_time = time.time()
        
        # Detect face first
        faces = self.detector.get(image)
        
        if len(faces) == 0:
            return {
                'mode': 'liveness',
                'is_live': False,
                'reason': 'no_face_detected',
                'processing_time': time.time() - start_time
            }
        
        face = faces[0]
        bbox = face.bbox.astype(int)
        
        # Check depth-based liveness
        if depth_image is not None:
            liveness_result = self._check_depth_liveness(depth_image, bbox)
        else:
            liveness_result = {
                'is_live': None,
                'reason': 'no_depth_data',
                'method': 'none'
            }
        
        processing_time = time.time() - start_time
        
        return {
            'mode': 'liveness',
            'bbox': bbox.tolist(),
            **liveness_result,
            'processing_time': processing_time
        }
    
    def _check_depth_liveness(self, depth_image: np.ndarray, bbox: np.ndarray) -> Dict:
        """Check liveness using depth variance"""
        x1, y1, x2, y2 = bbox
        
        # Extract face depth region
        face_depth = depth_image[y1:y2, x1:x2]
        
        # Filter out invalid depth values (0 or very far)
        valid_depth = face_depth[(face_depth > 0) & (face_depth < 5000)]
        
        if len(valid_depth) < 100:
            return {
                'is_live': False,
                'reason': 'insufficient_depth_data',
                'method': 'depth_variance'
            }
        
        # Calculate depth statistics
        depth_variance = np.var(valid_depth)
        depth_range = np.max(valid_depth) - np.min(valid_depth)
        depth_mean = np.mean(valid_depth)
        
        # Thresholds (tunable)
        min_variance = 100  # mm^2
        min_range = 10  # mm
        
        is_live = (depth_variance > min_variance) and (depth_range > min_range)
        
        return {
            'is_live': is_live,
            'confidence': float(min(depth_variance / 1000, 1.0)),  # Normalize
            'depth_variance': float(depth_variance),
            'depth_range': float(depth_range),
            'depth_mean': float(depth_mean),
            'method': 'depth_variance'
        }
    
    def match_face(self, embedding: np.ndarray, threshold: float) -> Optional[Dict]:
        """Match face embedding against database"""
        if not self.face_database:
            return None
        
        best_similarity = -1
        best_match = None
        
        for person_id, data in self.face_database.items():
            # Compare with all stored embeddings for this person
            for stored_embedding in data['embeddings']:
                similarity = np.dot(embedding, stored_embedding)
                
                if similarity > best_similarity:
                    best_similarity = similarity
                    best_match = {
                        'person_id': person_id,
                        'name': data['name'],
                        'similarity': float(similarity)
                    }
        
        if best_similarity >= threshold:
            # Update metadata
            if best_match['person_id'] in self.metadata:
                self.metadata[best_match['person_id']]['last_seen'] = time.strftime('%Y-%m-%d %H:%M:%S')
                self.metadata[best_match['person_id']]['encounter_count'] += 1
                self.save_metadata()
            
            return best_match
        
        return None
    
    def process(self, image: np.ndarray, params: Dict, depth_image: Optional[np.ndarray] = None) -> Dict:
        """Main processing function"""
        mode = params.get('mode', 'detect_recognize')
        
        if mode == 'detect':
            return self.detect_faces(image, params)
        elif mode == 'recognize':
            return self.recognize_face(image, params)
        elif mode == 'detect_recognize':
            return self.detect_and_recognize(image, params)
        elif mode == 'register':
            return self.register_face(image, params)
        elif mode == 'liveness':
            return self.check_liveness(image, depth_image, params)
        elif mode == 'analyze':
            # Full analysis: detect + recognize + liveness
            result = self.detect_and_recognize(image, params)
            if depth_image is not None and result['num_faces'] > 0:
                liveness = self.check_liveness(image, depth_image, params)
                result['liveness'] = liveness
            return result
        else:
            return {'error': f'Unknown mode: {mode}'}
    
    def visualize(self, image: np.ndarray, result: Dict, params: Dict) -> np.ndarray:
        """Visualize results on image"""
        vis_image = image.copy()
        mode = result.get('mode', 'detect_recognize')
        
        if 'faces' in result:
            for face in result['faces']:
                bbox = face['bbox']
                x1, y1, x2, y2 = map(int, bbox)
                
                # Determine color based on recognition
                if face.get('recognized', False):
                    color = (0, 255, 0)  # Green for recognized
                    label = face.get('name', 'Unknown')
                    if 'similarity' in face:
                        label += f" ({face['similarity']:.2f})"
                else:
                    color = (0, 165, 255)  # Orange for unknown
                    label = "Unknown"
                
                # Draw bbox
                cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 2)
                
                # Draw label
                cv2.putText(vis_image, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                # Draw landmarks if available
                if face.get('landmarks'):
                    landmarks = np.array(face['landmarks'], dtype=np.int32)
                    for point in landmarks:
                        cv2.circle(vis_image, tuple(point), 2, (0, 255, 255), -1)
                
                # Draw attributes
                y_offset = y2 + 20
                if 'age' in face:
                    cv2.putText(vis_image, f"Age: {face['age']}", (x1, y_offset),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    y_offset += 20
                if 'gender' in face:
                    cv2.putText(vis_image, f"Gender: {face['gender']}", (x1, y_offset),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Draw liveness result
        if 'liveness' in result:
            liveness = result['liveness']
            if liveness.get('is_live') is not None:
                status = "LIVE" if liveness['is_live'] else "SPOOF"
                color = (0, 255, 0) if liveness['is_live'] else (0, 0, 255)
                cv2.putText(vis_image, f"Liveness: {status}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        return vis_image
    
    def get_info(self) -> Dict:
        """Get model information"""
        return {
            'model': 'InsightFace',
            'modes': ['detect', 'recognize', 'detect_recognize', 'register', 'liveness', 'analyze'],
            'database_size': len(self.face_database),
            'total_samples': sum(len(data['embeddings']) for data in self.face_database.values()),
            'registered_people': list(self.metadata.keys())
        }
