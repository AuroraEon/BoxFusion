from __future__ import annotations

import re

from boxfusion.template_grounding.types import TemplateSpec


TEMPLATE_CATALOG = (
    TemplateSpec(
        template_id="go_from_room_to_room",
        intent="go_from_room_to_room",
        pattern=re.compile(r"^go from (?P<room_a>.+?) to (?P<room_b>.+)$"),
        slot_names=("room_a", "room_b"),
        description='Matches "go from <room_a> to <room_b>".',
    ),
    TemplateSpec(
        template_id="go_to_room_with_object",
        intent="go_to_room_with_object",
        pattern=re.compile(r"^go to the room with (?P<object_label>.+)$"),
        slot_names=("object_label",),
        description='Matches "go to the room with <object_label>".',
    ),
    TemplateSpec(
        template_id="go_to_anchor",
        intent="go_to_anchor",
        pattern=re.compile(r"^go to anchor (?P<anchor_id>.+)$"),
        slot_names=("anchor_id",),
        description='Matches "go to anchor <anchor_id>".',
    ),
    TemplateSpec(
        template_id="go_to_the_room",
        intent="go_to_room",
        pattern=re.compile(r"^go to the (?P<room>.+)$"),
        slot_names=("room",),
        description='Matches "go to the <room>".',
    ),
    TemplateSpec(
        template_id="go_to_room",
        intent="go_to_room",
        pattern=re.compile(r"^go to (?P<room>.+)$"),
        slot_names=("room",),
        description='Matches "go to <room>".',
    ),
)
