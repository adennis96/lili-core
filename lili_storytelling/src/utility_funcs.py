from __future__ import print_function
import sys
from nltk.stem.snowball import SnowballStemmer

stemmer = SnowballStemmer('english')

def eprint(*args, **kwargs):
	'''
	Prints to stderr
	'''
	print(*args, file=sys.stderr, **kwargs)

def get_target(self, s, targets, targets_syn = None):  
	'''
	Attempts to match 's' to one of the targets, either by an exact match or
	by matching s to one of the lists of target synonyms.
	'''

	s = s.lower()
      
    for target in targets:
        if s == target.lower():
            return target
    
    s = list(map(lambda i: stemmer.stem(i), s.split()))

    for word in s:
    	if word in targets:
    		return word

    if targets_syn is not None
		# Applies stemmer.stem() to each member of the nested List target_syns
		targets_syn = list(map(lambda syn_list: 
			list(map(lambda syn: stemmer.stem(syn), syn_list)), targets_syn))

    	for word in s:
        	for syn_list in targets_syn:
            	if word in syn_list:
                	return targets[targets_syn.index(syn_list)]

    return None