"""Reduce rigid assembly constraints to weld forests without locking outside joints."""


class RigidConnections:
    def __init__(self, body_ids):
        self.parents = {body_id: body_id for body_id in body_ids}

    def root(self, body_id):
        root = body_id
        while self.parents[root] != root:
            root = self.parents[root]
        while self.parents[body_id] != body_id:
            parent = self.parents[body_id]
            self.parents[body_id] = root
            body_id = parent
        return root

    def connect(self, base, follower):
        base_root, follower_root = self.root(base), self.root(follower)
        if base_root == follower_root:
            return False
        self.parents[follower_root] = base_root
        return True


def normalize_rigid_connections(bodies, joints, groups):
    """Return equivalent constraints and diagnostics for redundant/locked joints."""
    body_map = {body['uid']: body for body in bodies}
    if len(body_map) != len(bodies):
        raise ValueError('Exported body identifiers must be unique.')
    connections = RigidConnections(body_map)
    result, diagnostics = [], []
    for joint in joints:
        if joint['type'] == 'Weld' and not connections.connect(joint['base'], joint['follower']):
            diagnostics.append(dict(name=joint['name'], status='redundant',
                                    reason='Bodies are already connected by rigid constraints.'))
        else:
            result.append(joint)
    for group in groups:
        members = sorted(set(group['members']))
        unknown = set(members)-body_map.keys()
        if unknown:
            raise ValueError(f"Rigid group {group['name']} references missing bodies: {sorted(unknown)}")
        if len(members) < 2:
            diagnostics.append(dict(name=group['name'], status='excluded',
                                    reason='Rigid group has fewer than two exported solid bodies.'))
            continue
        base = members[0]
        for follower in members[1:]:
            if connections.connect(base, follower):
                result.append(dict(name=group['name']+'/Weld_'+follower, type='Weld',
                                   base=base, follower=follower, frame=list(body_map[base]['transform'])))
    active = []
    for joint in result:
        if joint['type'] != 'Weld' and connections.root(joint['base']) == connections.root(joint['follower']):
            diagnostics.append(dict(name=joint['name'], status='locked', type=joint['type'],
                                    reason='Both endpoints belong to the same rigid assembly; no relative motion remains.'))
        else:
            active.append(joint)
    return active, diagnostics
