Import("env")

from pathlib import Path


def patch_websockets_client() -> None:
    project_dir = Path(env["PROJECT_DIR"])
    libdeps_dir = project_dir / ".pio" / "libdeps"
    if not libdeps_dir.exists():
        return

    target = None
    for path in libdeps_dir.rglob("WebSocketsClient.cpp"):
        if path.parent.name == "src" and path.parent.parent.name.lower() == "websockets":
            target = path
            break

    if target is None or not target.exists():
        return

    old_line = "    return begin(host.toString().c_str(), port, url, protocol);"
    new_block = (
        "    char hostStr[16] = {0};\n"
        "    snprintf(hostStr, sizeof(hostStr), \"%u.%u.%u.%u\", host[0], host[1], host[2], host[3]);\n"
        "    return begin(hostStr, port, url, protocol);"
    )

    text = target.read_text(encoding="utf-8")
    if old_line in text:
        target.write_text(text.replace(old_line, new_block), encoding="utf-8")
        print("[patch_websockets] Applied Teensy IPAddress compatibility patch")


def patch_websockets_header() -> None:
    project_dir = Path(env["PROJECT_DIR"])
    libdeps_dir = project_dir / ".pio" / "libdeps"
    if not libdeps_dir.exists():
        return

    target = None
    for path in libdeps_dir.rglob("WebSockets.h"):
        if path.parent.name == "src" and path.parent.parent.name.lower() == "websockets":
            target = path
            break

    if target is None or not target.exists():
        return

    old_include = "#include <Ethernet.h>"
    new_include = (
        "#if defined(ARDUINO_TEENSY41) || defined(ARDUINO_TEENSY40)\n"
        "#include <NativeEthernet.h>\n"
        "#else\n"
        "#include <Ethernet.h>\n"
        "#endif"
    )

    text = target.read_text(encoding="utf-8")
    marker = "#elif(WEBSOCKETS_NETWORK_TYPE == NETWORK_W5100)"
    if marker in text and old_include in text:
        text = text.replace(old_include, new_include)
        target.write_text(text, encoding="utf-8")
        print("[patch_websockets] Applied NativeEthernet include patch")


patch_websockets_client()
patch_websockets_header()
