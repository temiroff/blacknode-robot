# Profiles

Component of `blacknode-robot`.

Node sources for this component belong in this folder. Until they move here,
nodes claim the component inline:

    @node(name="MyNode", component="profiles", ...)

Once sources live here, declare the folder in `blacknode-package.toml`:

    [components.profiles]
    nodes = ["components/profiles/nodes"]

and the inline `component=` argument can be dropped — the loader infers it
from the directory.
