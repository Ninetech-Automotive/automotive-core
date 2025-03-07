import pytest
from Navigation.Edge import Edge
from Navigation.EdgeStatus import EdgeStatus

def test_get_weight_unknown_status():
    edge = Edge()
    edge.status = EdgeStatus.UNKNOWN
    edge.length = 1
    assert edge.get_weight() == 150

def test_get_weight_obstructed_status():
    edge = Edge()
    edge.status = EdgeStatus.OBSTRUCTED
    edge.length = 2
    assert edge.get_weight() == 150

def test_get_weight_free_status():
    edge = Edge()
    edge.status = EdgeStatus.FREE
    edge.length = 3
    assert edge.get_weight() == 130

def test_get_weight_potentially_obstructed_status():
    edge = Edge()
    edge.status = EdgeStatus.POTENTIALLY_OBSTRUCTED
    edge.length = 4
    assert edge.get_weight() == 160

def test_get_weight_potentially_free_status():
    edge = Edge()
    edge.status = EdgeStatus.POTENTIALLY_FREE
    edge.length = 5
    assert edge.get_weight() == 160

def test_get_weight_custom_length():
    edge = Edge()
    edge.status = EdgeStatus.FREE
    edge.length = 7
    assert edge.get_weight() == 170

def test_get_weight_zero_length():
    edge = Edge()
    edge.status = EdgeStatus.UNKNOWN
    edge.length = 0
    assert edge.get_weight() == 140

def test_get_weight_negative_length():
    edge = Edge()
    edge.status = EdgeStatus.OBSTRUCTED
    edge.length = -1
    assert edge.get_weight() == 120

def test_get_weight_large_length():
    edge = Edge()
    edge.status = EdgeStatus.FREE
    edge.length = 1000
    assert edge.get_weight() == 10100
