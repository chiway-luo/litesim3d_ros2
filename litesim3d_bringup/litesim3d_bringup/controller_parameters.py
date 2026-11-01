"""ROS launch ParameterFile adapter for native controller geometry preparation."""
from pathlib import Path
import tempfile

from launch_ros.parameter_descriptions import ParameterFile
import yaml

from .model import _request, load_yaml


def model_geometry(robot_description, base_link, requests):
    return _request("model_geometry", description=robot_description, base_link=base_link, requests=requests)


def resolve_controller_parameters(configuration, robot_description, base_link='base_link'):
    return _request("controller_parameters", configuration=configuration,
                    description=robot_description, base_link=base_link)


class UrdfControllerParameters(ParameterFile):
    """Launch parameter file with deferred paths and standard temporary-file cleanup."""
    def __init__(self, param_file, robot_description, base_link='base_link'):
        self._generated = None
        super().__init__(param_file)
        self._robot_description = robot_description
        self._base_link = base_link

    def evaluate(self, context):
        if self._generated is None:
            source = super().evaluate(context)
            values = resolve_controller_parameters(load_yaml(source.read_text()),
                                                   self._robot_description, self._base_link)
            try:
                with tempfile.NamedTemporaryFile(mode='w', prefix='litesim-controllers-', suffix='.yaml',
                                                 encoding='utf-8', delete=False) as stream:
                    self._generated = Path(stream.name)
                    yaml.safe_dump(values, stream, sort_keys=False)
            except BaseException:
                self.cleanup()
                raise
        return self._generated

    def cleanup(self):
        if self._generated is not None:
            self._generated.unlink(missing_ok=True)
            self._generated = None
        super().cleanup()
