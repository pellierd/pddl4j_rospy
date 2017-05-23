#!/usr/bin/env python
import copy


class Fluents:
    """ This class implements a Fluent. A Fluent is composed by :
    - a predicate (a str)
    - a list of args (also str in the list)
    """

    def __init__(self, predicat=None, liste_args=None):
        """Constructor of the class"""

        if predicat is not None \
            and liste_args is not None \
            and isinstance(predicat, str) \
            and isinstance(liste_args, str):
            self._predicat = predicat
            self._list_arg = list()
            self._list_arg.append(liste_args)

        elif predicat is not None \
        and liste_args is not None \
        and isinstance(predicat, str) \
        and isinstance(liste_args, list) \
        and Fluents.is_list_str(liste_args):
            self._predicat = predicat
            self._list_arg = liste_args

        elif predicat is not None \
            and liste_args is None \
            and isinstance(predicat, str):
            self._predicat = predicat
            self._list_arg = list()

        else:
            raise SyntaxError("Error in constructor in class Fluents, there is no predicat")

    def copy(self):
        """
        This will act as the copy constructor.
        Work like that : b = a.copy()
        if the object is none then it will create one
        :return: the object
        """
        if self is None:
            return Fluents()
        else:
            return copy.deepcopy(self)

    def _get_predicat(self):
        """
        Return the predicate
        """
        return self._predicat

    def _get_list_arg(self):
        """
        Return the list of args
        """
        return self._list_arg

    def _set_predicat(self, new_predicat):
        """
        Set the predicate
        :param new_predicat: the value we want to set in the Fluents for the predicate
        """
        if isinstance(new_predicat, str):
            self._predicat = new_predicat
        else:
            raise SyntaxError("Error in function _set_predicat in class Fluents.py,"
                              "The value in parameter is not a str")

    def _set_list_arg(self, liste):
        """
        Set the list of Args
        :param liste: the list we want to set in the Fluents for the list of Args
        """
        if Fluents.is_list_str(liste):
            self._list_arg = liste
        elif isinstance(liste, str):
            self._list_arg = list()
            self._list_arg.append(liste)
        else:
            raise SyntaxError("Error in function _set_list_args in class Fluents.py"
                              "The value in parameter is not a str or a list of str")

    def add_args(self, liste):
        """
        Add an args or a list of args to the current list of args
        :param liste: an args or a list of args we want to add ot the current list of args
        :return: <code>True</code> if we added correctly the arg(s), <code>False</code> otherwise
        """

        if Fluents.is_list_str(liste):
            for elem in liste:
                self._list_arg.append(elem)
        else:
            raise SyntaxError("Error in function add_args in class Fluents.py, one of the parameters"
                              "of the class is not set to the right type")

    def remise_a_zero_args(self):
        """
        Clear the list of args
        """
        self._list_arg = list()

    def affiche_fluent(self):
        """
        Display the fluent
        """
        i = 0
        print "predicat : " + self._predicat
        for elem in self._list_arg:
            print "element " + repr(i) + " de la liste : " + elem
            i += 1

    # This is the properties, that protect the use of the _gets and _sets of the class
    predicat = property(_get_predicat, _set_predicat)
    liste_args = property(_get_list_arg, _set_list_arg)

    @staticmethod
    def is_list_str(liste):
        """
        Check if the list contains only str
        :param liste: the ist that needs to be ckecked
        :return: <code>True</code> is the liste constains only str, <code>False</code> otherwise
        """

        for elem in liste:
            if not isinstance(elem, str):
                return False
        return True


if __name__ == '__main__':

    # Test of the class Fluent

    a = Fluents("on")
    a.affiche_fluent()

    a._set_list_arg(["a", "b"])

    a.affiche_fluent()
    a.add_args(["c", "d"])

    a.affiche_fluent()
