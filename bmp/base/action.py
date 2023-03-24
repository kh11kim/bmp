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
    #is_fwd: Optional[bool] = field(default_factory=lambda :True)
    #is_fwd: Optional[bool] = field(default_factory=lambda :True)
    #prev_stage: Optional[int] = field(default_factory=lambda :None)

    def reverse(self):
        raise NotImplementedError()
    
    @property
    def robot_name(self):
        if self.name == "pick":
            return self.parent_to
        elif self.name == "place":
            return self.parent_from
    
    def as_string(self):
        return f"{self.name}-{self.obj_name}-{self.parent_from}-{self.parent_to}"

@dataclass
class Pick(Attach):
    name: Optional[bool] = field(default_factory=lambda :"pick")
    #is_fwd: Optional[bool] = field(default_factory=lambda :True)

    def reverse(self):
        return Place(
            obj_name=self.obj_name,
            parent_from=self.parent_to,
            parent_to=self.parent_from)

@dataclass
class Place(Attach):
    name: Optional[bool] = field(default_factory=lambda :"place")
    #is_fwd: Optional[bool] = field(default_factory=lambda :True)

    def reverse(self):
        return Pick(
            obj_name=self.obj_name,
            parent_from=self.parent_to,
            parent_to=self.parent_from)
    