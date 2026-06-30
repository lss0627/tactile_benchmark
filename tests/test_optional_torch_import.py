import importlib
import importlib.util
import tomllib


def test_torch_is_declared_as_optional_train_extra_not_base_dependency():
    with open("pyproject.toml", "rb") as stream:
        pyproject = tomllib.load(stream)

    dependencies = pyproject["project"]["dependencies"]
    train_extra = pyproject["project"]["optional-dependencies"]["train"]

    assert not any(dep.lower().startswith("torch") for dep in dependencies)
    assert any(dep.lower().startswith("torch") for dep in train_extra)


def test_base_training_modules_import_without_torch():
    import isaac_tactile_libero.training.config as train_config
    import isaac_tactile_libero.training.checkpoint as checkpoint

    assert train_config.TrainingConfig is not None
    assert checkpoint.summarize_checkpoint is not None
    if importlib.util.find_spec("torch") is None:
        assert importlib.import_module("isaac_tactile_libero.training.config") is train_config
