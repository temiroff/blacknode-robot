# Adapters

Transport adapters for the `contracts` component of `blacknode-robot`.

One folder per transport, each mirroring the component layout:

    adapters/ros2/nodes/
    adapters/ros2/templates/

Declare it in `blacknode-package.toml`:

    [components.contracts.adapters.ros2]
    description = "ROS 2 adapter for contracts."
    default = false
    capabilities = ["adapter.contracts.ros2"]
    nodes = ["components/contracts/adapters/ros2/nodes"]

Adapters stay `default = false`: the capability package owns them, and
`blacknode-ros2` provides only the shared transport underneath.
