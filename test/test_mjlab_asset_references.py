import importlib
import pkgutil

import pytest


def test_all_exported_mjlab_entity_specs_compile() -> None:
    pytest.importorskip("mjlab", exc_type=ImportError)
    import robot_learning_lab_zoo.assets.mjlab as package
    from mjlab.entity.entity import EntityCfg

    seen_spec_functions = set()
    compiled_configs = []
    for module_info in pkgutil.iter_modules(package.__path__, package.__name__ + "."):
        module = importlib.import_module(module_info.name)
        for name, value in vars(module).items():
            if not name.endswith("_CFG") or not isinstance(value, EntityCfg):
                continue
            if value.spec_fn in seen_spec_functions:
                continue
            seen_spec_functions.add(value.spec_fn)
            model = value.spec_fn().compile()
            assert model.nbody > 1, f"{module_info.name}.{name} compiled without articulated bodies"
            compiled_configs.append(f"{module_info.name}.{name}")

    assert len(compiled_configs) == 26
