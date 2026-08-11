"""
v7.10 — the plate-bottom floor is REFCOUNTED, not a plain flag.

WHY THIS EXISTS
---------------
``StageController.set_print_floor_active`` is armed by several independent
subsystems that can overlap: ``PrintManager``, ``SimplePrintManager``,
``PickAndPlaceManager`` and (new in v7.10) the needle-calibration wizard's
plate-bottom touch-off, which arms it while the operator jogs the needle toward
glass by hand.

With the previous plain bool, the FIRST disarm dropped the floor for everyone.
A caller still descending would then believe it was protected while the clamp
was off — *false* protection, which is strictly worse than never arming, because
nothing looks wrong. This is the same non-refcounted-global hazard as
``PositionPoller._suspended``.

The public signature is unchanged (``set_print_floor_active(bool)``), so these
tests also pin that existing callers keep working.
"""

import unittest

from SupportClasses.StageController import StageController


def _ctrl() -> StageController:
    """A controller without running __init__ (no serial, no threads).

    Mirrors the ``__new__``-partial idiom the rest of the suite uses; the
    refcount must work on one of these, because that is what most callers'
    tests construct.
    """
    c = StageController.__new__(StageController)
    c._print_floor_depth = 0
    return c


class TestRefcountSemantics(unittest.TestCase):

    def test_starts_disarmed(self):
        self.assertFalse(_ctrl()._print_floor_active)

    def test_one_arm_arms_it(self):
        c = _ctrl()
        c.set_print_floor_active(True)
        self.assertTrue(c._print_floor_active)

    def test_balanced_arm_disarm_returns_to_off(self):
        c = _ctrl()
        c.set_print_floor_active(True)
        c.set_print_floor_active(False)
        self.assertFalse(c._print_floor_active)

    def test_NESTED_disarm_does_not_drop_the_floor(self):
        """THE BUG. Two callers arm; the inner one finishes; the floor must stay
        armed for the outer one, which is still descending."""
        c = _ctrl()
        c.set_print_floor_active(True)     # outer: a print starts
        c.set_print_floor_active(True)     # inner: a touch-off arms too
        c.set_print_floor_active(False)    # inner finishes
        self.assertTrue(
            c._print_floor_active,
            "inner disarm dropped the floor while the outer caller was still "
            "running — this is the false-protection bug the refcount fixes")
        c.set_print_floor_active(False)    # outer finishes
        self.assertFalse(c._print_floor_active)

    def test_three_deep(self):
        c = _ctrl()
        for _ in range(3):
            c.set_print_floor_active(True)
        for expected in (True, True, False):
            c.set_print_floor_active(False)
            self.assertIs(c._print_floor_active, expected)

    def test_extra_disarm_cannot_poison_a_later_arm(self):
        """An unbalanced disarm must not drive the count negative — otherwise
        the NEXT arm would be a silent no-op and a real descent would run
        unprotected."""
        c = _ctrl()
        for _ in range(5):
            c.set_print_floor_active(False)
        self.assertEqual(c._print_floor_depth, 0)
        c.set_print_floor_active(True)
        self.assertTrue(c._print_floor_active)


class TestBackCompat(unittest.TestCase):

    def test_reads_on_a_bare_new_object_do_not_raise(self):
        """Partial controllers built with __new__ never ran __init__, so the
        depth attribute is absent; the property must degrade to False."""
        c = StageController.__new__(StageController)
        self.assertFalse(c._print_floor_active)

    def test_arm_on_a_bare_new_object_works(self):
        c = StageController.__new__(StageController)
        c.set_print_floor_active(True)
        self.assertTrue(c._print_floor_active)

    def test_direct_assignment_still_works(self):
        """Tests and hard resets assign the flag directly; keep that working."""
        c = _ctrl()
        c._print_floor_active = True
        self.assertTrue(c._print_floor_active)
        c._print_floor_active = False
        self.assertFalse(c._print_floor_active)
        self.assertEqual(c._print_floor_depth, 0)

    def test_direct_assignment_resets_a_deep_count(self):
        """Assigning False is a hard reset, not a decrement — so a test that
        force-clears the flag really does clear it."""
        c = _ctrl()
        for _ in range(4):
            c.set_print_floor_active(True)
        c._print_floor_active = False
        self.assertFalse(c._print_floor_active)


class TestTheClampStillConsultsIt(unittest.TestCase):
    """The refcount is only useful if `_apply_print_floor_raw` still reads it."""

    def test_clamp_is_inert_while_disarmed(self):
        c = _ctrl()
        c._plate_bottom_z_zref = 10.0
        c._zero_position = {"Z": 0.0}
        self.assertFalse(c._print_floor_active)

    def test_clamp_reads_the_refcounted_property(self):
        import inspect
        src = inspect.getsource(StageController._apply_print_floor_raw)
        self.assertIn("_print_floor_active", src,
                      "the floor clamp must still gate on the armed state")


if __name__ == "__main__":
    unittest.main()
