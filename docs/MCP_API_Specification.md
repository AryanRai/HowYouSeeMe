# MCP API Specification for HowYouSeeMe

## Overview

This document defines the Model Context Protocol (MCP) API for HowYouSeeMe, providing standardized interfaces for LLM agents (primarily Ally) to interact with the world perception system.

## API Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    MCP Server Architecture                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐            │
│  │   FastAPI   │  │ WebSocket   │  │ Tool Calling│            │
│  │   Server    │  │ Streaming   │  │ Framework   │            │
│  └─────────────┘  └─────────────┘  └─────────────┘            │
│         │                │                │                    │
│         └────────────────┼────────────────┘                    │
│                          │                                     │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │            World State Manager                          │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │   │
│  │  │   Entity    │  │   Memory    │  │  Resource   │    │   │
│  │  │  Manager    │  │   System    │  │  Manager    │    │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘    │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Core MCP Tools

### 1. World State Query Tools

#### `get_world_state`
**Description**: Get current world state entities within specified parameters

**Parameters**:
```json
{
  "spatial_query": {
    "type": "circle|bbox|all",
    "center": [x, y, z],
    "radius": 2.0,
    "bbox": [[x1, y1, z1], [x2, y2, z2]]
  },
  "entity_filters": {
    "types": ["object", "human", "place"],
    "classes": ["apple", "person", "table"],
    "confidence_min": 0.5,
    "time_window": "last_10_minutes"
  },
  "include_details": {
    "attributes": true,
    "relations": true,
    "detection_history": false,
    "evidence_images": false
  },
  "max_results": 50
}
```

**Response**:
```json
{
  "success": true,
  "query_timestamp": "2025-09-24T08:39:38Z",
  "total_entities": 12,
  "entities": [
    {
      "entity_id": "obj-00017",
      "type": "object",
      "class": "apple",
      "confidence": 0.86,
      "position": [2.341, -0.12, 0.76],
      "last_seen": "2025-09-24T08:35:20Z",
      "attributes": {
        "color": "red",
        "size": "small",
        "status": "on_table"
      },
      "spatial_relations": [
        {
          "type": "on_top_of",
          "target": "place-table-01",
          "confidence": 0.92
        }
      ],
      "memory_flags": ["remembered_by_user"]
    }
  ],
  "summary": "Found 12 entities: 8 objects, 3 places, 1 human. Notable: red apple on kitchen table (remembered)."
}
```

#### `get_entity_details`
**Description**: Get comprehensive details for a specific entity

**Parameters**:
```json
{
  "entity_id": "obj-00017",
  "include_evidence": true,
  "include_history": true,
  "include_relations": true
}
```

**Response**:
```json
{
  "success": true,
  "entity": {
    "entity_id": "obj-00017",
    "type": "object",
    "class": "apple",
    "confidence": 0.86,
    "first_seen": "2025-09-24T07:45:12Z",
    "last_seen": "2025-09-24T08:35:20Z",
    "pose": {
      "frame": "map",
      "position": [2.341, -0.12, 0.76],
      "covariance": [0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05]
    },
    "detection_history": [
      {
        "timestamp": "2025-09-24T08:35:20Z",
        "confidence": 0.86,
        "bbox": [245, 180, 85, 92],
        "camera_id": "kinect_front"
      }
    ],
    "evidence_images": [
      "/media/crops/obj-00017/20250924_083520.jpg"
    ],
    "provenance": {
      "detectors": ["yolo-v8", "sam-vit-h"],
      "model_versions": {
        "yolo": "v8.0.196"
      }
    }
  }
}
```

### 2. Memory Operations

#### `remember_entity`
**Description**: Store an entity in persistent memory with user annotation

**Parameters**:
```json
{
  "entity_id": "obj-00017",
  "user_note": "This is the apple I want to eat for lunch",
  "memory_type": "user_request",
  "priority": "high",
  "tags": ["food", "lunch", "personal"]
}
```

**Response**:
```json
{
  "success": true,
  "memory_id": "mem-uuid-12345",
  "message": "Apple (obj-00017) remembered with note: 'This is the apple I want to eat for lunch'",
  "memory_summary": "Red apple on kitchen table at position [2.341, -0.12, 0.76], remembered for lunch",
  "evidence_stored": true
}
```

#### `query_memory`
**Description**: Search memories using natural language or structured queries

**Parameters**:
```json
{
  "query": "Where is the apple I wanted for lunch?",
  "query_type": "natural_language",
  "search_scope": {
    "time_range": "last_24_hours",
    "memory_types": ["user_request", "episodic"],
    "spatial_radius": 5.0
  },
  "max_results": 10,
  "include_evidence": true
}
```

**Response**:
```json
{
  "success": true,
  "query_understanding": {
    "intent": "spatial_location_query",
    "entities": ["apple"],
    "temporal_context": "lunch",
    "spatial_context": "location_query"
  },
  "results": [
    {
      "memory_id": "mem-uuid-12345",
      "relevance_score": 0.94,
      "entity_id": "obj-00017",
      "memory_type": "user_request",
      "user_note": "This is the apple I want to eat for lunch",
      "current_location": [2.341, -0.12, 0.76],
      "last_verified": "2025-09-24T08:35:20Z",
      "evidence_image": "/media/crops/obj-00017/20250924_083520.jpg",
      "spatial_description": "On the kitchen table, near the center"
    }
  ],
  "answer": "The apple you wanted for lunch is still on the kitchen table at position [2.34, -0.12, 0.76]. It was last seen 4 minutes ago and appears to be in the same location where you asked me to remember it."
}
```

#### `forget_memory`
**Description**: Remove a memory from the system

**Parameters**:
```json
{
  "memory_id": "mem-uuid-12345",
  "reason": "No longer needed"
}
```

### 3. Scene Understanding

#### `describe_scene`
**Description**: Get natural language description of current scene or region

**Parameters**:
```json
{
  "spatial_query": {
    "type": "bbox",
    "bbox": [[0, 0, 0], [5, 5, 2]]
  },
  "detail_level": "comprehensive",
  "focus": {
    "include_activities": true,
    "include_relationships": true,
    "include_emotions": false,
    "include_objects": true,
    "include_people": true
  },
  "use_vlm": true
}
```

**Response**:
```json
{
  "success": true,
  "scene_description": {
    "summary": "The kitchen scene shows an active cooking environment with one person and various objects arranged on surfaces.",
    "detailed_description": "A person is standing in the kitchen near the stove, actively engaged in cooking. On the kitchen table, there's a red apple positioned near the center, along with what appears to be cooking utensils. The person seems focused on their cooking task, with their attention directed toward the stove area. The overall environment suggests meal preparation is underway.",
    "entities_mentioned": ["human-12", "obj-00017", "place-table-01", "place-stove-01"],
    "activities_detected": ["cooking", "food_preparation"],
    "spatial_relationships": [
      "Apple on kitchen table",
      "Person near stove",
      "Utensils on counter"
    ],
    "generated_by": "vlm_gpt4_vision",
    "confidence": 0.89
  }
}
```

#### `analyze_changes`
**Description**: Identify and describe changes in the environment over time

**Parameters**:
```json
{
  "time_comparison": {
    "baseline": "30_minutes_ago",
    "current": "now"
  },
  "spatial_region": "all",
  "change_types": ["object_moved", "person_entered", "object_added", "object_removed"],
  "significance_threshold": 0.3
}
```

### 4. Spatial Reasoning

#### `spatial_query`
**Description**: Answer spatial reasoning questions about the environment

**Parameters**:
```json
{
  "question": "What objects are within reach of the person cooking?",
  "context": {
    "reference_entities": ["human-12"],
    "spatial_constraints": {
      "max_distance": 1.5,
      "accessibility": "arm_reach"
    }
  }
}
```

**Response**:
```json
{
  "success": true,
  "spatial_analysis": {
    "reference_entity": "human-12",
    "reference_position": [1.8, 0.5, 0.0],
    "query_region": {
      "type": "sphere",
      "center": [1.8, 0.5, 1.2],
      "radius": 1.5
    },
    "reachable_objects": [
      {
        "entity_id": "obj-00023",
        "class": "spatula",
        "distance": 0.8,
        "accessibility": "easy_reach"
      },
      {
        "entity_id": "obj-00017",
        "class": "apple",
        "distance": 1.2,
        "accessibility": "moderate_reach"
      }
    ],
    "answer": "The person cooking can easily reach a spatula (0.8m away) and could reach the apple on the table with moderate effort (1.2m away). There are 2 objects within comfortable reaching distance."
  }
}
```

### 5. System Control

#### `trigger_model`
**Description**: Request specific model execution on current or specified frame

**Parameters**:
```json
{
  "model_name": "yolo_detection",
  "priority": "high",
  "roi": {
    "bbox": [100, 100, 300, 200]
  },
  "parameters": {
    "confidence_threshold": 0.5,
    "classes_of_interest": ["apple", "orange", "banana"]
  }
}
```

#### `get_system_status`
**Description**: Get current system health and resource usage

**Response**:
```json
{
  "success": true,
  "system_status": {
    "overall_health": "healthy",
    "perception_pipeline": {
      "slam_status": "running",
      "fps": 29.5,
      "map_quality": "good"
    },
    "resource_usage": {
      "gpu_utilization": 0.65,
      "ram_usage": "3.2GB",
      "active_models": ["slam", "person_detector"],
      "queued_requests": 0
    },
    "entity_statistics": {
      "total_entities": 45,
      "active_entities": 12,
      "remembered_entities": 3
    },
    "last_update": "2025-09-24T08:39:35Z"
  }
}
```

## WebSocket Streaming API

### Real-time Entity Updates

**Connection**: `ws://localhost:8000/ws/entity_stream`

**Subscribe Message**:
```json
{
  "action": "subscribe",
  "filters": {
    "entity_types": ["object", "human"],
    "spatial_region": {
      "type": "bbox",
      "bbox": [[0, 0, 0], [5, 5, 2]]
    },
    "update_types": ["entity_created", "entity_updated", "entity_removed"]
  }
}
```

**Stream Updates**:
```json
{
  "event_type": "entity_updated",
  "timestamp": "2025-09-24T08:39:38Z",
  "entity_id": "obj-00017",
  "changes": {
    "position": {
      "old": [2.341, -0.12, 0.76],
      "new": [2.355, -0.15, 0.76]
    },
    "confidence": {
      "old": 0.86,
      "new": 0.89
    }
  },
  "entity_snapshot": {
    "entity_id": "obj-00017",
    "class": "apple",
    "position": [2.355, -0.15, 0.76],
    "confidence": 0.89
  }
}
```

## Sample Implementation (Python)

```python
from fastapi import FastAPI, WebSocket
from typing import Optional, List, Dict, Any
import asyncio
import json

app = FastAPI(title="HowYouSeeMe MCP Server")

class MCPServer:
    def __init__(self):
        self.world_state_manager = WorldStateManager()
        self.memory_system = MemorySystem()
        self.resource_manager = ResourceManager()
        
    @app.post("/mcp/get_world_state")
    async def get_world_state(
        self,
        spatial_query: Optional[Dict] = None,
        entity_filters: Optional[Dict] = None,
        include_details: Optional[Dict] = None,
        max_results: int = 50
    ):
        """Get current world state entities"""
        try:
            # Parse spatial query
            entities = await self.world_state_manager.query_entities(
                spatial_query=spatial_query,
                filters=entity_filters,
                max_results=max_results
            )
            
            # Generate summary
            summary = self._generate_entity_summary(entities)
            
            return {
                "success": True,
                "query_timestamp": datetime.utcnow().isoformat(),
                "total_entities": len(entities),
                "entities": entities,
                "summary": summary
            }
            
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "error_type": type(e).__name__
            }
    
    @app.post("/mcp/remember_entity")
    async def remember_entity(
        self,
        entity_id: str,
        user_note: Optional[str] = None,
        memory_type: str = "user_request",
        priority: str = "medium",
        tags: Optional[List[str]] = None
    ):
        """Store entity in persistent memory"""
        try:
            memory_id = await self.memory_system.remember_entity(
                entity_id=entity_id,
                user_note=user_note,
                memory_type=memory_type,
                priority=priority,
                tags=tags or []
            )
            
            entity = await self.world_state_manager.get_entity(entity_id)
            
            return {
                "success": True,
                "memory_id": memory_id,
                "message": f"{entity.class.title()} ({entity_id}) remembered with note: '{user_note}'",
                "memory_summary": f"{entity.attributes.get('color', '')} {entity.class} at position {entity.pose.position}",
                "evidence_stored": True
            }
            
        except Exception as e:
            return {
                "success": False,
                "error": str(e)
            }
    
    @app.post("/mcp/query_memory")
    async def query_memory(
        self,
        query: str,
        query_type: str = "natural_language",
        search_scope: Optional[Dict] = None,
        max_results: int = 10,
        include_evidence: bool = True
    ):
        """Search memories using natural language"""
        try:
            # Parse natural language query
            query_understanding = await self._parse_nl_query(query)
            
            # Search memories
            results = await self.memory_system.semantic_search(
                query=query,
                scope=search_scope,
                max_results=max_results
            )
            
            # Generate natural language answer
            answer = await self._generate_memory_answer(query, results)
            
            return {
                "success": True,
                "query_understanding": query_understanding,
                "results": results,
                "answer": answer
            }
            
        except Exception as e:
            return {
                "success": False,
                "error": str(e)
            }
    
    @app.websocket("/ws/entity_stream")
    async def entity_stream(self, websocket: WebSocket):
        """WebSocket endpoint for real-time entity updates"""
        await websocket.accept()
        
        try:
            # Get subscription filters
            subscribe_msg = await websocket.receive_json()
            filters = subscribe_msg.get("filters", {})
            
            # Start streaming updates
            async for update in self.world_state_manager.stream_updates(filters):
                await websocket.send_json(update)
                
        except Exception as e:
            await websocket.send_json({
                "error": str(e),
                "event_type": "error"
            })
        finally:
            await websocket.close()

# Tool registration for Ally integration
MCP_TOOLS = {
    "get_world_state": {
        "name": "get_world_state",
        "description": "Get current world state entities with spatial and temporal filtering",
        "parameters": {
            "type": "object",
            "properties": {
                "spatial_query": {"type": "object"},
                "entity_filters": {"type": "object"},
                "max_results": {"type": "integer", "default": 50}
            }
        }
    },
    "remember_entity": {
        "name": "remember_entity", 
        "description": "Store an entity in persistent memory with user annotation",
        "parameters": {
            "type": "object",
            "properties": {
                "entity_id": {"type": "string"},
                "user_note": {"type": "string"},
                "priority": {"type": "string", "enum": ["low", "medium", "high"]}
            },
            "required": ["entity_id"]
        }
    },
    "query_memory": {
        "name": "query_memory",
        "description": "Search memories using natural language queries",
        "parameters": {
            "type": "object", 
            "properties": {
                "query": {"type": "string"},
                "max_results": {"type": "integer", "default": 10}
            },
            "required": ["query"]
        }
    }
}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

## Integration with Ally

The MCP server automatically registers with Ally's tool calling framework:

```python
# In Ally's tool registry
async def register_howyouseeme_tools():
    """Register HowYouSeeMe MCP tools with Ally"""
    mcp_client = MCPClient("http://localhost:8000")
    
    for tool_name, tool_spec in MCP_TOOLS.items():
        ally_tool_registry.register_tool(
            name=f"world_perception_{tool_name}",
            spec=tool_spec,
            endpoint=f"http://localhost:8000/mcp/{tool_name}"
        )
```

This comprehensive MCP API provides Ally and other LLM agents with powerful world understanding capabilities through standardized, reliable interfaces.