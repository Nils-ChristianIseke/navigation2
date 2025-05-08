import dagger
import anyio
import sys
from dagger import dag, function


async def main():
    async with dagger.Connection(dagger.Config(log_output=sys.stderr)) as client:
        build = client.host().directory(".").docker_build()
        build.with_exec(["git", "clone"])
        result = await build.with_exec(["echo", "Hello from container"]).stdout()
        await build.export("/tmp/nav2dagger.tar")

@function
async def build():
    async with dagger.Connection(dagger.Config(log_output=sys.stdout)) as client:
        file = client.host().file("/tmp/nav2dagger.tar")
        dag.container().import_(file).terminal()
        # return dag.container().with_exec(["echo", "Hello from container"]).stdout()


if __name__ == "__main__":

    # anyio.run(main)
    anyio.run(build)