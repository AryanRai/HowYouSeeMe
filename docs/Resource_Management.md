# Resource Management & Entity-Centric Architecture

## Overview

HowYouSeeMe implements intelligent resource management to handle multiple computer vision models efficiently while maintaining real-time performance. This document outlines the orchestration strategy, entity-centric data model, and fusion rules.

## Resource Management Strategy

### Always-On Components
- **SLAM**: Continuous localization and mapping (GPU/CPU hybrid)
- **Lightweight Person Detector**: CPU-based, 1-2Hz for triggering heavy models
- **TF Publisher**: Transform tree and camera poses

### On-Demand Components (GPU-Managed)
- **YOLO Object Detection**: Triggered by queries or scheduled sweeps
- **Human Pose/Body Tracking**: Activated when person detector signals
- **Face Analysis Pipeline**: Launched when face detector finds faces
- **Semantic Segmentation**: ROI-based processing
- **Vision-Language Models**: High-cost, LLM-request only

### Resource Orchestration Logic

```python
class ResourceManager:
    def __init__(self):
        self.gpu_busy = False
        self.model_containers = {}
        self.job_queue = asyncio.Queue()
    
    async def request_model(self, model_name, priority=1, roi=None):
        """Request model execution with priority and ROI"""
        if model_name in self.HEAVY_MODELS and self.gpu_busy:
            await self.job_queue.put({
                'model': model_name, 
                'priority': priority, 
                'roi': roi
            })
        else:
            await self.launch_model(model_name, roi)
    
    async def launch_model(self, model_name, roi=None):
        """Launch containerized model"""
        if model_name in self.HEAVY_MODELS:
            self.gpu_busy = True
        
        container = await self.start_container(model_name)
        self.model_containers[model_name] = container
        
        # Auto-shutdown after idle timeout
        asyncio.create_task(
            self.auto_shutdown(model_name, idle_timeout=30)
        )

    async def auto_shutdown(self, model_name, idle_timeout):
        """Auto-shutdown idle models"""
        await asyncio.sleep(idle_timeout)
        if self.is_model_idle(model_name):
            await self.release_model(model_name)
```

### Trigger Heuristics

1. **Object Detection Triggers**:
   - LLM queries about objects
   - Person detector signals
   - Scheduled low-frequency sweeps (every 15 seconds when idle)
   - Memory operations requiring object anchoring

2. **Face Pipeline Triggers**:
   - Cheap face detector (Haar/MTCNN) finds face candidates
   - LLM queries about people or emotions
   - Security/monitoring mode activation

3. **Segmentation Triggers**:
   - Object detection provides ROI
   - LLM requests detailed scene understanding
   - Memory operations requiring precise boundaries

4. **VLM Triggers**:
   - LLM requests scene description
   - Complex visual reasoning queries
   - Scene captioning for memory storage

## Entity-Centric Data Model

### Core Entity Structure

```json
{
  "entity_id": "obj-00017",
  "type": "object",
  "class": "apple",
  "confidence": 0.86,
  "first_seen": "2025-09-15T13:02:17Z",
  "last_seen": "2025-09-15T13:05:40Z",
  
  "pose": {
    "frame": "map",
    "position": [2.341, -0.12, 0.76],
    "orientation_quat": [0, 0, 0, 1],
    "covariance": [0.05, 0, 0, 0, 0, 0, 
                  0, 0.05, 0, 0, 0, 0, 
                  0, 0, 0.05, 0, 0, 0]
  },
  
  "detection_history": [
    {
      "camera_id": "kinect_front",
      "frame": "camera_rgb",
      "bbox": [x, y, w, h],
      "depth_median": 0.76,
      "timestamp": "2025-09-15T13:05:40.123Z",
      "confidence": 0.86
    }
  ],
  
  "visual_embedding_id": "emb-9853",
  "segmentation_mask_id": "mask-221",
  
  "attributes": {
    "color": "red",
    "size": "small",
    "status": "on_table",
    "material": "organic"
  },
  
  "spatial_relations": [
    {
      "type": "on_top_of",
      "target": "place-table-01",
      "confidence": 0.92
    },
    {
      "type": "near",
      "target": "human-12",
      "distance": 0.8,
      "confidence": 0.85
    }
  ],
  
  "memory_flags": ["remembered_by_user"],
  "memory_notes": ["User asked to remember apple location"],
  
  "provenance": {
    "detectors": ["yolo-v8", "sam-vit-h"],
    "evidence_images": ["/media/crops/obj-00017/20250915_130540.jpg"],
    "model_versions": {
      "yolo": "v8.0.196",
      "sam": "vit-h-4b8939"
    }
  },
  
  "metadata": {
    "created_by": "perception_pipeline",
    "last_updated": "2025-09-15T13:05:40Z",
    "update_count": 12
  }
}
```

### Human Entity Extensions

```json
{
  "entity_id": "human-12",
  "type": "human",
  "class": "person",
  
  "identity": {
    "face_id": "face-9834",
    "name": "Unknown_Person_1",
    "recognition_confidence": 0.75,
    "last_face_update": "2025-09-15T13:05:40Z"
  },
  
  "pose_tracking": {
    "skeleton_points": {...},
    "current_pose": "standing",
    "pose_confidence": 0.91,
    "tracking_id": "track-557"
  },
  
  "face_analysis": {
    "emotion": {
      "primary": "happy",
      "confidence": 0.67,
      "all_emotions": {"happy": 0.67, "neutral": 0.23, "surprised": 0.10}
    },
    "gaze_direction": {
      "vector": [0.2, -0.1, 0.8],
      "target_estimate": "obj-00017",
      "confidence": 0.74
    },
    "age_estimate": {"range": "25-35", "confidence": 0.42},
    "gender_estimate": {"prediction": "female", "confidence": 0.89}
  },
  
  "activity_analysis": {
    "current_activity": "cooking",
    "confidence": 0.78,
    "activity_duration": 120.5,
    "activity_history": ["walking", "standing", "cooking"]
  },
  
  "interaction_context": {
    "attention_objects": ["obj-00017", "place-stove-01"],
    "social_context": "alone",
    "engagement_level": "focused"
  }
}
```

## Fusion Rules & Entity Management

### Temporal Association

```python
def fuse_detection_to_entity(detection, existing_entities):
    """Fuse new detection with existing entities"""
    
    # Project detection to map coordinates
    map_point, cov = project_to_map(
        detection.bbox, 
        detection.depth, 
        detection.camera_pose
    )
    
    # Find candidate entities within adaptive radius
    radius = adaptive_radius(cov, detection.class)
    candidates = find_entities_within_radius(map_point, radius)
    
    if candidates:
        # Score candidates based on multiple factors
        best_entity = score_association_candidates(
            candidates, detection, map_point
        )
        
        if best_entity.association_score > ASSOCIATION_THRESHOLD:
            # Update existing entity
            update_entity_with_detection(best_entity, detection, map_point)
            return best_entity
    
    # Create new entity
    return create_new_entity(detection, map_point, cov)

def score_association_candidates(candidates, detection, map_point):
    """Score entity association candidates"""
    scored_candidates = []
    
    for entity in candidates:
        score = 0.0
        
        # Spatial proximity score (0-0.4)
        spatial_dist = euclidean_distance(entity.pose.position, map_point)
        spatial_score = max(0, 0.4 * (1 - spatial_dist / MAX_ASSOCIATION_RADIUS))
        
        # Class consistency score (0-0.3)
        class_score = 0.3 if entity.class == detection.class else 0.0
        
        # Temporal continuity score (0-0.2)
        time_diff = abs(detection.timestamp - entity.last_seen)
        temporal_score = max(0, 0.2 * (1 - time_diff / MAX_TEMPORAL_GAP))
        
        # Visual similarity score (0-0.1)
        if entity.visual_embedding_id and detection.visual_embedding:
            similarity = cosine_similarity(
                entity.visual_embedding, 
                detection.visual_embedding
            )
            visual_score = 0.1 * similarity
        else:
            visual_score = 0.0
        
        total_score = spatial_score + class_score + temporal_score + visual_score
        scored_candidates.append((entity, total_score))
    
    return max(scored_candidates, key=lambda x: x[1])
```

### Multi-Model Fusion

```python
def fuse_multi_modal_detection(detections_dict):
    """Fuse detections from multiple models"""
    
    fused_entities = []
    
    # Group detections by spatial proximity
    detection_clusters = spatial_clustering(detections_dict)
    
    for cluster in detection_clusters:
        fused_entity = {}
        
        # Confidence-weighted attribute fusion
        for model_name, detection in cluster.items():
            weight = MODEL_WEIGHTS[model_name]
            
            # Fuse class predictions
            if 'class' in fused_entity:
                fused_entity['class'] = weighted_vote(
                    fused_entity['class'], detection.class, weight
                )
            else:
                fused_entity['class'] = detection.class
            
            # Fuse bounding boxes (intersection over union)
            if 'bbox' in fused_entity:
                fused_entity['bbox'] = merge_bboxes(
                    fused_entity['bbox'], detection.bbox, weight
                )
            else:
                fused_entity['bbox'] = detection.bbox
            
            # Aggregate confidences
            if 'confidence' in fused_entity:
                fused_entity['confidence'] = confidence_fusion(
                    fused_entity['confidence'], detection.confidence, weight
                )
            else:
                fused_entity['confidence'] = detection.confidence
        
        fused_entities.append(fused_entity)
    
    return fused_entities
```

## Memory System Integration

### Two-Tier Memory Architecture

**Short-term (Redis):**
- Active entities (rolling window: 10 minutes or 1000 entities)
- Fast retrieval for real-time LLM queries
- Automatic expiration and cleanup

**Long-term (PostgreSQL + Vector DB):**
- Persistent memories with semantic embeddings
- Evidence storage (images, masks, metadata)
- Episodic event reconstruction

### Memory Operations

```python
class MemorySystem:
    def __init__(self):
        self.redis_client = redis.Redis()
        self.postgres_client = PostgresClient()
        self.vector_db = VectorDB()
    
    async def remember_entity(self, entity_id, user_note=None):
        """Store entity in long-term memory"""
        entity = await self.get_entity(entity_id)
        
        # Create memory record
        memory_record = {
            'memory_id': f"mem-{uuid4()}",
            'entity_id': entity_id,
            'timestamp': datetime.utcnow(),
            'map_pose': entity.pose,
            'user_note': user_note,
            'evidence_images': entity.provenance.evidence_images,
            'semantic_embedding': await self.generate_embedding(entity),
            'text_summary': await self.generate_summary(entity)
        }
        
        # Store in both systems
        await self.postgres_client.insert_memory(memory_record)
        await self.vector_db.insert_embedding(
            memory_record['memory_id'],
            memory_record['semantic_embedding']
        )
        
        # Mark entity as remembered
        entity.memory_flags.append('remembered_by_user')
        await self.update_entity(entity)
        
        return memory_record['memory_id']
    
    async def semantic_search(self, query, max_results=10):
        """Search memories using natural language"""
        query_embedding = await self.embed_text(query)
        
        # Vector similarity search
        similar_memories = await self.vector_db.similarity_search(
            query_embedding, max_results
        )
        
        # Retrieve full records
        memory_records = []
        for memory_id, similarity in similar_memories:
            record = await self.postgres_client.get_memory(memory_id)
            record['similarity'] = similarity
            memory_records.append(record)
        
        return memory_records
```

## Performance Optimizations

### ROI-Based Processing
- Use object detection bounding boxes as ROIs for segmentation
- Crop and resize images before feeding to expensive models
- Batch multiple ROIs for efficient GPU utilization

### Adaptive Model Selection
- Use lighter models when GPU is constrained
- Fallback to CPU models with reduced resolution/FPS
- Dynamic quality scaling based on system load

### Caching & Precomputation
- Cache model outputs for static scenes
- Precompute embeddings for known objects
- Smart invalidation based on scene changes

### Batch Processing
- Group similar operations (multiple face crops → batch face recognition)
- Temporal batching for non-real-time models
- Asynchronous processing with result queues

This resource management strategy ensures efficient utilization of computational resources while maintaining real-time performance and rich world understanding capabilities.