#!/usr/bin/env python3
"""
Tier 5 — MCP Server
Exposes robot world state to LLMs via Model Context Protocol.
"""
from mcp.server import Server
from mcp.server.stdio import stdio_server
import json
from pathlib import Path
import asyncio

server = Server("howyouseeme")

@server.tool()
async def query_world(filter: str = "") -> str:
    """Returns current world state — all visible objects, people, robot position, and recent events"""
    try:
        data = json.loads(Path("/tmp/world_state.json").read_text())
    except FileNotFoundError:
        return json.dumps({"error": "World state not available"})
    
    if not filter:
        return json.dumps(data, indent=2)
    
    # Filter results
    result = {
        "generated_at": data.get("generated_at"),
        "robot": data.get("robot"),
        "objects": {},
        "people": {},
        "recent_events": []
    }
    
    filter_lower = filter.lower()
    
    for k, v in data.get("objects", {}).items():
        if filter_lower in v.get("label", "").lower():
            result["objects"][k] = v
    
    for k, v in data.get("people", {}).items():
        result["people"][k] = v
    
    for event in data.get("recent_events", []):
        if filter_lower in event.get("summary", "").lower():
            result["recent_events"].append(event)
    
    return json.dumps(result, indent=2)

@server.tool()
async def where_is(label: str) -> str:
    """Find the last known position of a specific object or person"""
    try:
        data = json.loads(Path("/tmp/world_state.json").read_text())
    except FileNotFoundError:
        return json.dumps({"found": False, "error": "World state not available"})
    
    label_lower = label.lower()
    
    # Search objects
    for k, v in data.get("objects", {}).items():
        if label_lower in v.get("label", "").lower():
            return json.dumps({
                "found": True,
                "source": "live_detection",
                "label": v["label"],
                "position": v["position"],
                "last_seen": v["last_seen"],
                "confidence": v["confidence"]
            })
    
    # Search people
    for k, v in data.get("people", {}).items():
        if label_lower in "person":
            return json.dumps({
                "found": True,
                "source": "live_detection",
                "label": "person",
                "position": v["position"],
                "last_seen": v["last_seen"],
                "identity": v.get("identity", "unknown")
            })
    
    # Search named memories
    for k, v in data.get("named_memories", {}).items():
        if label_lower in k.lower() or label_lower in v.get("label", "").lower():
            return json.dumps({
                "found": True,
                "source": "named_memory",
                "name": k,
                "label": v["label"],
                "position": v["position"],
                "last_confirmed": v["last_confirmed"],
                "status": v["status"]
            })
    
    return json.dumps({"found": False, "label": label})

@server.tool()
async def remember_object(name: str, label: str, hint: str = "") -> str:
    """Ask the robot to track and pin a specific object's location"""
    # TODO: Call ROS2 service /memory/remember
    return json.dumps({
        "success": False,
        "message": "Service integration not yet implemented. Add memory manually to /tmp/named_memories.json"
    })

@server.tool()
async def recall_memory(name: str) -> str:
    """Retrieve a previously remembered object's current location"""
    try:
        memories = json.loads(Path("/tmp/named_memories.json").read_text())
    except FileNotFoundError:
        return json.dumps({"found": False, "error": "No memories stored"})
    
    if name in memories:
        return json.dumps({
            "found": True,
            **memories[name]
        })
    
    return json.dumps({"found": False, "name": name})

@server.tool()
async def forget_memory(name: str) -> str:
    """Stop tracking a named memory"""
    # TODO: Call ROS2 service /memory/forget
    return json.dumps({
        "success": False,
        "message": "Service integration not yet implemented"
    })

@server.tool()
async def get_recent_events(event_type: str = "", limit: int = 10) -> str:
    """Retrieve recent events from short-term memory"""
    try:
        data = json.loads(Path("/tmp/world_state.json").read_text())
    except FileNotFoundError:
        return json.dumps({"error": "World state not available"})
    
    events = data.get("recent_events", [])
    
    if event_type:
        events = [e for e in events if event_type.lower() in e.get("event_type", "").lower()]
    
    return json.dumps(events[:limit], indent=2)

@server.tool()
async def get_checkpoint(checkpoint_id: str) -> str:
    """Retrieve a specific past checkpoint frame and its analysis"""
    checkpoint_path = Path(f"/tmp/stm/{checkpoint_id}")
    
    if not checkpoint_path.exists():
        return json.dumps({"error": "Checkpoint not found"})
    
    result = {}
    
    try:
        if (checkpoint_path / "meta.json").exists():
            result["meta"] = json.loads((checkpoint_path / "meta.json").read_text())
        
        if (checkpoint_path / "enriched.json").exists():
            result["enriched"] = json.loads((checkpoint_path / "enriched.json").read_text())
        
        if (checkpoint_path / "detections.json").exists():
            result["detections"] = json.loads((checkpoint_path / "detections.json").read_text())
        
        if (checkpoint_path / "pose.json").exists():
            result["pose"] = json.loads((checkpoint_path / "pose.json").read_text())
        
        return json.dumps(result, indent=2)
    except Exception as e:
        return json.dumps({"error": str(e)})

if __name__ == "__main__":
    asyncio.run(stdio_server(server))
