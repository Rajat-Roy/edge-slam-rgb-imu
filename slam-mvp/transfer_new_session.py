#!/usr/bin/env python3
"""
Transfer new session data from Android device
"""
import subprocess
import os
from pathlib import Path

def check_adb_connection():
    """Check if ADB is connected"""
    try:
        result = subprocess.run(['adb', 'devices'], capture_output=True, text=True)
        if 'device' in result.stdout and 'offline' not in result.stdout:
            print("✅ ADB device connected")
            return True
        else:
            print("❌ No ADB device found")
            return False
    except FileNotFoundError:
        print("❌ ADB not found - install Android SDK tools")
        return False

def list_slam_sessions():
    """List available SLAM sessions on device"""
    try:
        result = subprocess.run([
            'adb', 'shell', 'ls', '-la', '/storage/emulated/0/SLAM/'
        ], capture_output=True, text=True)

        print("📱 Available SLAM sessions on device:")
        print(result.stdout)

        # Get list of session directories
        sessions = []
        for line in result.stdout.split('\n'):
            if 'slam_session_' in line and 'drw' in line:
                session_name = line.split()[-1]
                sessions.append(session_name)

        return sessions
    except Exception as e:
        print(f"❌ Error listing sessions: {e}")
        return []

def get_session_info(session_name):
    """Get information about a specific session"""
    try:
        result = subprocess.run([
            'adb', 'shell', 'cat', f'/storage/emulated/0/SLAM/{session_name}/session_metadata.json'
        ], capture_output=True, text=True)

        if result.returncode == 0:
            import json
            metadata = json.loads(result.stdout)
            duration = (metadata['endTime'] - metadata['startTime']) / 1000.0
            print(f"📊 {session_name}:")
            print(f"   Duration: {duration:.1f} seconds")
            print(f"   Frames: {metadata['totalFrames']}")
            print(f"   IMU samples: {metadata['totalImuSamples']}")
            return metadata
        else:
            print(f"❌ Could not read metadata for {session_name}")
            return None
    except Exception as e:
        print(f"❌ Error getting session info: {e}")
        return None

def transfer_session(session_name, local_path):
    """Transfer a session from device to local directory"""
    print(f"📥 Transferring {session_name}...")

    # Create local directory
    session_path = Path(local_path) / session_name
    session_path.mkdir(parents=True, exist_ok=True)

    try:
        # Transfer entire session directory
        result = subprocess.run([
            'adb', 'pull',
            f'/storage/emulated/0/SLAM/{session_name}',
            str(local_path)
        ], capture_output=True, text=True)

        if result.returncode == 0:
            print(f"✅ Successfully transferred {session_name}")
            print(f"📁 Location: {session_path}")

            # Check transferred files
            files = list(session_path.rglob('*'))
            print(f"📄 Transferred {len(files)} files")

            return True
        else:
            print(f"❌ Transfer failed: {result.stderr}")
            return False

    except Exception as e:
        print(f"❌ Transfer error: {e}")
        return False

def main():
    print("=== SLAM Session Transfer Tool ===")

    # Check ADB connection
    if not check_adb_connection():
        print("\n💡 To connect ADB:")
        print("   1. Enable USB debugging on your Android device")
        print("   2. Connect via USB or use wireless debugging")
        print("   3. Run: adb devices")
        return

    # List available sessions
    sessions = list_slam_sessions()

    if not sessions:
        print("❌ No SLAM sessions found on device")
        return

    print(f"\n📋 Found {len(sessions)} session(s)")

    # Get info for each session
    session_metadata = {}
    for session in sessions:
        metadata = get_session_info(session)
        if metadata:
            session_metadata[session] = metadata

    # Find the longest session
    if session_metadata:
        longest_session = max(session_metadata.keys(),
                            key=lambda s: session_metadata[s]['endTime'] - session_metadata[s]['startTime'])

        longest_duration = (session_metadata[longest_session]['endTime'] -
                          session_metadata[longest_session]['startTime']) / 1000.0

        print(f"\n🏆 Longest session: {longest_session} ({longest_duration:.1f}s)")

        # Ask user which session to transfer
        print(f"\nSelect session to transfer:")
        for i, session in enumerate(sessions):
            metadata = session_metadata.get(session, {})
            duration = (metadata.get('endTime', 0) - metadata.get('startTime', 0)) / 1000.0
            frames = metadata.get('totalFrames', 0)
            print(f"  {i+1}. {session} - {duration:.1f}s, {frames} frames")

        print(f"  0. Transfer all sessions")

        try:
            choice = input(f"\nEnter choice (1-{len(sessions)}, 0 for all): ").strip()

            if choice == '0':
                # Transfer all sessions
                for session in sessions:
                    transfer_session(session, "data/")
            else:
                # Transfer specific session
                idx = int(choice) - 1
                if 0 <= idx < len(sessions):
                    selected_session = sessions[idx]
                    if transfer_session(selected_session, "data/"):
                        # Update the collected_data symlink to point to newest session
                        collected_data_path = Path("data/collected_data")
                        if collected_data_path.exists():
                            collected_data_path.unlink()

                        collected_data_path.symlink_to(selected_session)
                        print(f"🔗 Updated collected_data -> {selected_session}")
                else:
                    print("❌ Invalid choice")

        except ValueError:
            print("❌ Invalid input")
        except KeyboardInterrupt:
            print("\n⚠️ Transfer cancelled")

if __name__ == "__main__":
    main()