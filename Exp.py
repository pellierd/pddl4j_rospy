
import copy
from Fluents import Fluents


class Exp:
    """
    This class implements an expression. An expression is used to encode preconditions
    (negative and positive) but also effects of the instantiated action.
    """

    def __init__(self, positives=None, negatives=None):
        """
            This is the construtor of our class, you can use it without instanciate
            the positives or the negatives or you can create one with predefined
            positives and negatives lists
        """
        if positives is not None \
                and negatives is not None \
                and isinstance(positives, list) \
                and isinstance(negatives, list):
            error = False
            for elemp in positives:
                if not isinstance(elemp, Fluents):
                    error = True
            for elemn in negatives:
                if not isinstance(elemn, Fluents):
                    error = True
            if not error:
                self._positives = positives
                self._negatives = negatives
            else:
                print ("Erreur un des elements de la liste n'est pas un Fluent")
        elif positives is not None \
                and negatives is not None \
                and isinstance(positives, Fluents) \
                and isinstance(negatives, Fluents):
            self._positives = list()
            self._negatives = list()

            self._positives.append(positives)
            self._negatives.append(negatives)
        else:
            self._positives = []
            self._negatives = []

    def copy(self):
        """
        This will act as the copy constructor.
        Work like that : b = a.copy()
        if the object is none then it will create one
        :return: the object
        """
        if self is None:
            return Exp()
        else:
            return copy.deepcopy(self)

    def _get_positives(self):
        """
        return the list of positives
        """
        return self._positives

    def _get_negatives(self):
        """
        return the list of negatives
        """
        return self._negatives

    def _set_positives(self, positives):
        """
        Positives's setter
            :param positives : the list we want to set for the positives
        """
        if isinstance(positives, Fluents):
            self._positives = positives
        else:
            print("L'argument pour le set positives n'est pas un Fluent, veuillez reesayer")

    def _set_negatives(self,negatives):
        """
        Negatives's setter
            :param negatives : the list we want to set for the negatives
        """
        if isinstance(negatives,Fluents):
            self._negatives = negatives
        else:
            print("L'argument pour le set negatives n'est pas un Fluent, veuillez reesayer")

    def add_one_positive(self, positive):
        """
        Add a positive to the list of positives
        :param positive ; a Fluent that we want to add to the list of positives
        """
        if isinstance(positive, Fluents):
            self._positives.append(positive)
        else:
            print("L'argument pour l'ajout n'est pas un fluent, veuillez reesayer")

    def add_positives(self, *positives):
        """
        Add positives to the list of positives
        :param positives : a list of positives that we want to add to the list of positives
        """
        i = 0
        for elem in positives:
            if isinstance(elem, Fluents):
                self._positives.append(elem)
            else:
                print("L'argument numero " + repr(i) + " n'est pas un fluent, il n'as donc pas ete ajoute ")

    def add_one_negative(self, negative):
        """
        Add a negative to the list of negatives
        :param negative : a Fluent that we want to add to the list of negatives
        """
        if isinstance(negative, Fluents):
            self._negatives.append(negative)
            self._negatives.append(negative)
        else:
            print("L'argument pour l'ajout n'est pas un fluent, veuillez reesayer")

    def add_negatives(self, *negatives):
        """
        Add negatives to the list of negatives
        :param negatives : a list of negatives that we want to add to the list of negatives
        """
        i = 0
        for elem in negatives:
            if isinstance(elem, Fluents):
                self._negatives.append(elem)
            else:
                print("L'argument numero " + repr(i) + " n'est pas un fluent, il n'as donc pas ete ajoute ")

    def is_empty(self):
        """
        Check if the lists are empty
        :return: True or False
        """
        if not self._positives and not self._negatives:
            return True
        else:
            return False

    def affiche(self):
        """
        Display the informations of the object
        """
        print ("Affichage de l'Exp \n")
        print ("Affichage des positives : ")
        for elem in self._positives:
            print ("----------------------")
            elem.affiche_fluent()

        print ("\n")
        print ("Affichage des negatives :")
        for elem in self._negatives:
            print ("----------------------")
            elem.affiche_fluent()

    # This is the properties, that protect the use of the _gets and _sets of the class

    positives = property(_get_positives, _set_positives)
    negatives = property(_get_negatives, _set_negatives)

if __name__ == '__main__':

    # Test of the Exp class

    # Creation of a positive Fluent
    positive = Fluents("on", "a", "c", "g")
    # Display of the recently created Fluent

    # Creation of a negative fluent
    negative = Fluents("handempty")

    # Creation of a list of positives and a list of negatives :
    # Creation dof two new positives
    positive2 = Fluents("on", " z", "e")
    positive3 = Fluents("under", "y", "u")
    positives_liste = [positive2, positive3]

    # Creation of two new negatives
    negative2 = Fluents("handempty")
    negative3 = Fluents("on", "k", "l")
    negatives_liste = [negative2, negative3]

    # Creation of the obecjt Exp with the lists
    exp_with_list = Exp(positives_liste, negatives_liste)
    exp_with_list.affiche()

