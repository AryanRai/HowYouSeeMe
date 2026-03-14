#!/usr/bin/env python3
"""
Tier 5 — MCP Server
Exposes robot world state to LLMs via Model Context Protocol (HTTP/SSE on port 8090).
Ally connects at: http://localhost:8090/mcp
"""
from mcp.server.fastmcp import FastMCP
import json
import base64
import pathlib
import asyncio

mcp = FastMCP(
    "howyouseeme",
    host="0.0.0.0",
    port=8090,
    streamable_http_path="/mcp",
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _world() -> dict:
    try:
        return json.loads(pathlib.Path("/tmp/world_state.json").read_text())
    except FileNotFoundError:
        return {}

# ---------------------------------------------------------------------------
# Existing tools
# ---------------------------------------------------------------------------

@mcp.tool()
async def query_world(filter: str = "") -> str:
    """Returns current world state — all visible objects, people, robot position, and recent events"""
    data = _world()
    if not data:
        return json.dumps({"error": "World state not available"})

    if not filter:
        return json.dumps(data, indent=2)

    filter_lower = filter.lower()
    result = {
        "generated_at": data.get("generated_at"),
        "robot": data.get("robot"),
        "objects": {},
        "people": {},
        "recent_events": [],
    }
    for k, v in data.get("objects", {}).items():
        if filter_lower in v.get("label", "").lower():
            result["objects"][k] = v
    for k, v in data.get("people", {}).items():
        result["people"][k] = v
    for event in data.get("recent_events", []):
        if filter_lower in event.get("summary", "").lower():
            result["recent_events"].append(event)
    return json.dumps(result, indent=2)


@mcp.tool()
async def where_is(label: str) -> str:
    """Find the last known position of a specific object or person"""
    data = _world()
    if not data:
        return json.dumps({"found": False, "error": "World state not available"})

    label_lower = label.lower()

    for k, v in data.get("objects", {}).items():
        if label_lower in v.get("label", "").lower():
            return json.dumps({
                "found": True, "source": "live_detection",
                "label": v["label"], "position": v["position"],
                "last_seen": v["last_seen"], "confidence": v["confidence"],
            })

    for k, v in data.get("people", {}).items():
        if label_lower in "person":
            return json.dumps({
                "found": True, "source": "live_detection",
                "label": "person", "position": v["position"],
                "last_seen": v["last_seen"], "identity": v.get("identity", "unknown"),
            })

    for k, v in data.get("named_memories", {}).items():
        if label_lower in k.lower() or label_lower in v.get("label", "").lower():
            return json.dumps({
                "found": True, "source": "named_memory",
                "name": k, "label": v["label"], "position": v["position"],
                "last_confirmed": v["last_confirmed"], "status": v["status"],
            })

    return json.dumps({"found": False, "label": label})


@mcp.tool()
async def remember_object(name: str, label: str, hint: str = "") -> str:
    """Ask the robot to track and pin a specific object's location"""
    return json.dumps({
        "success": False,
        "message": "Service integration not yet implemented. Add memory manually to /tmp/named_memories.json",
    })


@mcp.tool()
async def recall_memory(name: str) -> str:
    """Retrieve a previously remembered object's current location"""
    try:
        memories = json.loads(pathlib.Path("/tmp/named_memories.json").read_text())
    except FileNotFoundError:
        return json.dumps({"found": False, "error": "No memories stored"})
    if name in memories:
        return json.dumps({"found": True, **memories[name]})
    return json.dumps({"found": False, "name": name})


@mcp.tool()
async def forget_memory(name: str) -> str:
    """Stop tracking a named memory"""
    return json.dumps({"success": False, "message": "Service integration not yet implemented"})


@mcp.tool()
async def get_recent_events(event_type: str = "", limit: int = 10) -> str:
    """Retrieve recent events from short-term memory"""
    data = _world()
    if not data:
        return json.dumps({"error": "World state not available"})
    events = data.get("recent_events", [])
    if event_type:
        events = [e for e in events if event_type.lower() in e.get("event_type", "").lower()]
    return json.dumps(events[:limit], indent=2)


@mcp.tool()
async def get_checkpoint(checkpoint_id: str) -> str:
    """Retrieve a specific past checkpoint frame and its analysis"""
    checkpoint_path = pathlib.Path(f"/tmp/stm/{checkpoint_id}")
    if not checkpoint_path.exists():
        return json.dumps({"error": "Checkpoint not found"})
    result = {}
    try:
        for name in ("meta", "enriched", "detections", "pose"):
            f = checkpoint_path / f"{name}.json"
            if f.exists():
                result[name] = json.loads(f.read_text())
        return json.dumps(result, indent=2)
    except Exception as e:
        return json.dumps({"error": str(e)})

# ---------------------------------------------------------------------------
# New tools for Ally
# ---------------------------------------------------------------------------

@mcp.tool()
async def get_camera_frame() -> str:
    """Returns the latest RGB frame from the robot as a base64 JPEG for vision LLMs"""
    try:
        img_bytes = pathlib.Path("/tmp/latest_frame.jpg").read_bytes()
        return base64.b64encode(img_bytes).decode()
    except FileNotFoundError:
        return json.dumps({"error": "No frame available yet — world synthesiser may not be running"})


@mcp.tool()
async def get_robot_status() -> str:
    """Returns a concise natural language summary of robot state for Ally's Robot Mode"""
    try:
        data = json.loads(pathlib.Path("/tmp/world_state.json").read_text())
    except FileNotFoundError:
        return "Robot world state not available — system may not be running."

    robot = data.get("robot", {})
    objects = data.get("objects", {})
    people = data.get("people", {})
    memories = data.get("named_memories", {})
    sleeping = pathlib.Path("/tmp/robot_sleeping").exists()

    obj_list = ", ".join(set(v["label"] for v in objects.values())) or "nothing"
    people_count = len(people)
    mem_list = ", ".join(memories.keys()) or "none"
    pos = robot.get("position", [0, 0, 0])

    return (
        f"Robot is {'sleeping' if sleeping else 'active'}. "
        f"Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}). "
        f"Currently seeing: {obj_list}. "
        f"People nearby: {people_count}. "
        f"Tracked memories: {mem_list}."
    )


@mcp.tool()
async def get_robot_context() -> str:
    """Returns the system prompt context block for Ally's Robot Mode"""
    return (
        "You are connected to a physical robot running HowYouSeeMe — "
        "a ROS 2 perception system with a Kinect v2 RGB-D camera and "
        "ORB-SLAM3 SLAM. You can see the robot's environment, track objects, "
        "and manage spatial memories.\n\n"
        "Available capabilities:\n"
        "- query_world: see all current objects, people, and events\n"
        "- where_is(label): find any object's 3D location\n"
        "- remember_object(name, label): pin an object to track persistently\n"
        "- recall_memory(name): get a pinned object's current location\n"
        "- get_camera_frame: see what the robot currently sees (base64 JPEG)\n"
        "- get_robot_status: quick natural language status summary\n"
        "- get_recent_events: last N perception events with timestamps\n"
        "When asked about physical locations, always call query_world first."
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    mcp.run(transport="streamable-http")
