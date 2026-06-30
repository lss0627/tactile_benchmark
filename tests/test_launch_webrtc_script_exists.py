from pathlib import Path
import subprocess


def test_launch_webrtc_script_is_shell_syntax_valid_and_documented():
    script = Path("scripts/launch_isaacsim_webrtc_smoke.sh")

    result = subprocess.run(["bash", "-n", str(script)], text=True, capture_output=True)
    text = script.read_text(encoding="utf-8")

    assert result.returncode == 0, result.stderr
    assert "ISAACSIM_ROOT" in text
    assert "./isaac-sim.streaming.sh" in text
    assert "./runheadless.sh" in text
    assert "isaacsim isaacsim.exp.full.streaming --no-window" in text
    assert "49100" in text
    assert "47998" in text
    assert "host networking" in text
    assert "PressButton visual smoke preparation" in text
