import dagger
from dagger import dag, function, object_type, DefaultPath
from typing import Annotated

@object_type
class Navigation2:
    @function
    def showcase_dagger_debugging(self,source: Annotated[dagger.Directory, DefaultPath("/")]) -> dagger.Container:
        return (
            dag.container()
            .from_("ros:rolling")
            .terminal()
            .with_exec(["bash", "-c", "apt update -y"])
            .with_exec(["bash", "-c", "apt install -y iputils-ping"])
            .with_directory("/", source)
            .terminal()
        )
    @function
    def build_ros_packages(self, source: Annotated[dagger.Directory, DefaultPath("/")]) -> dagger.Container:
        return (
            self.showcase_dagger_debugging(source)
            .with_exec(["bash", "-c", "colcon build"])
        )
    @function
    def test_ros_packages(self, source: Annotated[dagger.Directory, DefaultPath("/")]) -> dagger.Container:
        return (
            self.build_ros_packages(source)
            .with_exec(["bash", "-c", "colcon test"])
            .with_terminal()
        )
    @function
    def run_pre_commit(self, source: Annotated[dagger.Directory, DefaultPath("/")]) -> dagger.Container:
        return (
            dag.container().from_("jfxs/pre-commit")
            .with_directory("/", source)
            .with_exec(["bash", "-c", "pre-commit run --all-files"])
        )
