from dataclasses import dataclass, field
from typing import Optional, List

@dataclass
class TAMPAction:
    #stage_from: int
    obj_name: str
    #name: str
    

@dataclass
class Attach(TAMPAction):
    parent_from: str #Optional[str] = field(default_factory=lambda :None)
    parent_to: str #Optional[str] = field(default_factory=lambda :None)
    name: Optional[str] = field(default_factory=lambda :None)
    is_fwd: Optional[bool] = field(default_factory=lambda :True)
    #is_fwd: Optional[bool] = field(default_factory=lambda :True)
    #prev_stage: Optional[int] = field(default_factory=lambda :None)

@dataclass
class Pick(Attach):
    name: Optional[bool] = field(default_factory=lambda :"pick")
    #is_fwd: Optional[bool] = field(default_factory=lambda :True)
    #stage_from: Optional[int] = field(default_factory=lambda :None)

@dataclass
class Place(Attach):
    name: Optional[bool] = field(default_factory=lambda :"place")
    #is_fwd: Optional[bool] = field(default_factory=lambda :True)
    #stage_from: Optional[int] = field(default_factory=lambda :None)