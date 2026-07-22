# Capabilities

Component of `blacknode-robot`.

Node sources for this component belong in this folder. Until they move here,
nodes claim the component inline:

    @node(name="MyNode", component="capabilities", ...)

Once sources live here, declare the folder in `blacknode-package.toml`:

    [components.capabilities]
    nodes = ["components/capabilities/nodes"]

and the inline `component=` argument can be dropped — the loader infers it
from the directory.
