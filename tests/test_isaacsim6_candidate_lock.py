from pathlib import Path


LOCK = Path("requirements/candidates/lock-py312-isaacsim-6.0.1.txt")
GUIDE = Path("requirements/candidates/isaac-sim-6.0.1-candidate.md")


def test_candidate_lock_pins_runtime_and_toolchain() -> None:
    text = LOCK.read_text(encoding="utf-8")
    assert "isaacsim==6.0.1.0" in text
    assert "torch==2.11.0+cu128" in text
    assert "pip==26.1.2" in text
    assert "setuptools==78.1.0" in text
    assert "wheel==0.47.0" in text
    assert " @ file:" not in text
    assert "-e " not in text


def test_candidate_guide_freezes_python_and_preserves_driver() -> None:
    text = GUIDE.read_text(encoding="utf-8")
    assert "Python 3.12.13" in text
    assert "550.144.03" in text
    assert "UNVALIDATED" in text
    assert "595.58.03" in text
    assert "OMNI_KIT_ACCEPT_EULA=YES" in text
