import dagger
from dagger import dag, function, object_type
from typing import Annotated



from dagger import dag, function, DefaultPath 
import sys

@object_type
class Navigation2:
    @function
    def prepare_container(self,source: Annotated[dagger.Directory, DefaultPath("/")]) -> dagger.Container:
        return (
            dag.container()
            .from_("ros:rolling")
            .with_exec(["bash", "-c", "source /opt/ros/rolling/setup.bash"])
            .terminal()
            .with_exec(["bash", "-c", "apt update -y"])
            .with_exec(["bash", "-c", "apt install -y iputils-ping"])
            .terminal()
            .with_directory("/", source)
        )
    @function
    def build_ros(self, source: Annotated[dagger.Directory, DefaultPath("/")]) -> dagger.Container:
        return (
            self.prepare_container(source)
            .with_exec(["bash", "-c", "colcon build"])
        )
    @function
    def test_ros(self, source: Annotated[dagger.Directory, DefaultPath("/")]) -> dagger.Container:
        return (
            self.build_ros(source)
            .with_exec(["bash", "-c", "colcon test"])
            .with_terminal()
        )
    @function
    def lint(self, source: Annotated[dagger.Directory, DefaultPath("/")]) -> dagger.Container:
        return (
            dag.container().from_("jfxs/pre-commit")
            .with_directory("/", source)
            .with_exec(["bash", "-c", "pre-commit run --all-files"])
        )
