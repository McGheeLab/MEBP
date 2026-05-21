"""
gui.onboarding — First-run wizard and progressive-disclosure help (v7.4.0-c).

When MEBP launches and no needle gauge is configured (i.e. the user has
never completed Hardware Setup), MainWindow shows :class:`OnboardingWizard`
modally before the main window appears. The wizard walks the user through
the minimum setup needed to unlock the rest of the app, then commits a
:class:`HardwareConfig` and starter ink library.
"""

from gui.onboarding.wizard import OnboardingWizard

__all__ = ["OnboardingWizard"]
