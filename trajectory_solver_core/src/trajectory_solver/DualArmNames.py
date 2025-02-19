from typing import List

from trajectory_solver.PoseTypes import HandEnum


class DualArmNames:
    def __init__(self, namespaces: List[HandEnum], names: List[str]) -> None:
        self._namespaces = namespaces
        self._names = names

        self._namespace_dict = self.create_name_dict()
        self._all_names_lst = self.create_name_list()

    # Properties
    @property
    def namespace_dict(self):
        return self._namespace_dict

    @property
    def all_names_lst(self):
        return self._all_names_lst
    
    def create_name_dict(self):
        names_dict = dict()
        for ns in self._namespaces:
            names_dict[ns] = list(ns.value + "_" + name for name in self._names)
        return names_dict
    
    def create_name_list(self):
        name_lst = []
        for _, item in self._namespace_dict.items():
            name_lst += item
        return name_lst