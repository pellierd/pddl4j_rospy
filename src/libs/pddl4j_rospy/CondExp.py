#!/usr/bin/env python
import copy
from Exp import Exp
from Fluents import Fluents


class CondExp:
    """
    This class allows to implements a conditional effects of an action.
    """

    def __init__(self, effects=None, conditions=None):
        """
        This is the constructor of our class
        :param conditions: should be an Exp
        :param effects: should be an Exp
        """
        if conditions is not None and effects is not None and isinstance(conditions, Exp) and isinstance(effects, Exp):
            self._conditions = conditions
            self._effects = effects
        elif conditions is None and effects is not None and isinstance(effects, Exp):
            self._conditions = Exp()
            self._effects = effects
        else:
            self._conditions = Exp()
            self._effects = Exp()

    def copy(self):
        """
        This is the copy function that replace a copy construtor in Java
        Do'nt work atm
        :return: a copy of the object
        """
        if self is None:
            return CondExp()
        else:
            return copy.deepcopy(self)

    def _get_conditions(self):
        """
        condition's getter
        """
        return self._conditions

    def _get_effects(self):
        """Effects's getter"""
        return self._effects

    def _set_conditions(self, conditions):
        """Conditions's setter
            :param conditions : The value we want to set for the condition of the CondExp"""
        if isinstance(conditions,Exp):
            self._conditions = conditions
        else:
            print ("Conditions can't be affected because it's not an Exp")

    def _set_effects(self, effects):
        """Effects's setter
           :param effects : The value we want to set for the effect of the CondExp
        """
        if isinstance(effects,Exp):
            self._effects = effects
        else:
            print ("Effects can't be affected because it's not an Exp")

    def affiche(self):
        """
        This function display the informations of the CondExp
        """
        print ("Display of the CondExp :\n")
        print ("conditions part : \n")
        self._conditions.affiche()
        print ("\n")
        print ("effects part : \n")
        self._effects.affiche()

    # This is the property, that protect the use of the _gets and _sets of the class
    conditions = property(_get_conditions,_set_conditions)
    effects = property(_get_effects,_set_effects)

if __name__ == '__main__':

    # Test of the class

    #  Creation of a CondExp with only effects
    fluent_positives_effects = Fluents("on", "a", "b")
    fluent_negatives_effects = Fluents("handempty", "j", "c")
    exp_effect = Exp(fluent_positives_effects, fluent_negatives_effects)
    cond_exp_with_only_effects = CondExp(exp_effect)

    # Creation of a CondExp with a conditions & a effects
    fluents_positives_conditions = Fluents("on", "y", "z")
    fluents_negatives_conditions = Fluents("on", "t", "n")
    exp_conditions = Exp(fluents_positives_conditions, fluents_negatives_conditions)
    cond_exp_conditions_effects = CondExp(exp_effect, exp_conditions)
    cond_exp_conditions_effects.affiche()









