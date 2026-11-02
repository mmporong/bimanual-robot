import math
from types import SimpleNamespace
from jdamr_cube_navigation.reverse_parking import remembered_blind_points

def _scan(rays):
    return SimpleNamespace(ranges=rays, angle_min=-.02, angle_increment=.01,
                           range_min=.28, range_max=12.)
def test_blind_memory_requires_newer_valid_free_space():
    old=_scan([math.nan, math.nan, .5, math.nan, math.nan])
    pose=(0.,0.,0.); laser=(0.,0.,0.); current=(.3,0.,0.)
    assert remembered_blind_points([(pose,old)],current,laser,.28)
    for rays, cleared in [([1.]*5,True), ([math.inf]*5,False),
                          ([math.nan]*5,False), ([.4]*5,False),
                          ([.51]*5,False), ([1.,1.,.5,1.,1.],False)]:
        found=remembered_blind_points([(pose,old),(pose,_scan(rays))],current,laser,.28)
        assert bool(found) != cleared, (rays,found)
    # The same point cannot be cleared after it has entered the blind range.
    assert remembered_blind_points([(pose,old),(current,_scan([1.]*5))],current,laser,.28)
    # An obstacle observed again after free-space evidence is retained.
    assert remembered_blind_points([(pose,old),(pose,_scan([1.]*5)),(pose,old)],current,laser,.28)
    # A 90-degree vehicle rotation uses the newer laser frame for its ray evidence.
    rot=SimpleNamespace(ranges=[1.]*5,angle_min=-math.pi/2-.02,angle_increment=.01,
                        range_min=.28,range_max=12.)
    assert not remembered_blind_points([(pose,old),((0.,0.,math.pi/2),rot)],current,laser,.28)
