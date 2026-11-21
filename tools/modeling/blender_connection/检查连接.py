import asyncio, os, shutil
from pathlib import Path
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


async def main():
    server = os.environ.get("BLENDER_MCP_SERVER") or shutil.which("mcp-for-blender")
    if not server:
        raise RuntimeError(
            "Install the optional blender-mcp environment or set BLENDER_MCP_SERVER"
        )
    params = StdioServerParameters(
        command=str(server),
        args=["--host", "127.0.0.1", "--port", "19876"],
        env={"BLENDER_MCP_DISABLE_TELEMETRY": "1", "BLENDER_MCP_SAFE_MODE": "1"},
    )
    async with stdio_client(params) as (reader, writer), ClientSession(
        reader, writer
    ) as client:
        await client.initialize()
        response = await client.call_tool("get_scene_info", {"fields": ["size"]})
        result = "\n".join(
            item.text for item in response.content if item.type == "text"
        )
        print(result)
        if response.isError:
            raise SystemExit(1)


asyncio.run(main())
