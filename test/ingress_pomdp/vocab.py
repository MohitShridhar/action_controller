#!/usr/bin/env python

import numpy as np
import math

class Vocab(object):

    def __init__(self, file_path, T=15, V=10510):
        with open(file_path, 'r') as f:
            self.idx_to_token = f.readlines()
            self.token_to_idx = self.build_token_map()
            self.T = T
            self.V = V

    def build_token_map(self):
        token_to_idx = dict()
        for i in range(len(self.idx_to_token)):
            token_to_idx[self.idx_to_token[i].rstrip()] = i
        return token_to_idx

    def get_token(self, idx):
        if idx >= 0 and idx < len(self.idx_to_token)-1:
            return self.idx_to_token[idx+1].rstrip()
        return None

        
    def get_idx(self, token):
        if token in self.token_to_idx:
            return self.token_to_idx[token]
        return -1
    
        
    def get_vocab_size(self):
        return len(self.idx_to_token)


    def softmax(self, probs):
        softmax_inputs = np.array(probs)
        dense_softmax = np.zeros_like(probs)
        shifted_inputs = softmax_inputs - softmax_inputs.max() 
        exp_outputs = np.exp(shifted_inputs)
        exp_outputs_sum = exp_outputs.sum()
        if math.isnan(exp_outputs_sum):
          dense_softmax = exp_outputs * float('nan')
        assert exp_outputs_sum > 0
        if math.isinf(exp_outputs_sum):
          dense_softmax = np.zeros_like(exp_outputs)
        eps_sum = 1e-20
        dense_softmax = exp_outputs / max(exp_outputs_sum, eps_sum)
        return dense_softmax

    
    def probs_to_captions(self, word_probs):
        captions = []
        for b in range(len(word_probs)):
            sentence = ""
            for t in range(self.T):
                softmax_probs = self.softmax(word_probs[b][t])
                ml_idx = np.argmax(softmax_probs)
                sentence = sentence + self.get_token(ml_idx).rstrip() + " "
            captions.append(sentence)
        return captions


    def encode_one_hot(self, query):
        words = query.split()
        one_hot_seq = np.zeros((self.V, self.T), dtype=float)
        for t_idx in range(self.T):
            if t_idx < len(words):
                idx = self.get_idx(words[t_idx].strip().rstrip())
                one_hot_seq[idx, t_idx] = 1
            else:
                one_hot_seq[self.V-1, t_idx] = 1
        return one_hot_seq


    def cross_entropy(self, y, yhat):
        if yhat == 1:
            return -math.log(y)
        else:
            return -math.log(1-y)

    def cross_entropy_loss(self, y, yhat):
        assert(len(y)==len(yhat))
        return sum(self.cross_entropy(y[i], yhat[i]) for i in range(len(y)))
    

    def avg_cross_entropy_loss(self, probs, query):
        one_hot_seq = self.encode_one_hot(query)
        words = query.split()
        avg_losses = []
        check_range = len(words)

        captions = self.probs_to_captions(probs)
        
        for o in range(len(probs)):
            loss = 0
            caption = captions[o]
            check_range = caption.split().index("<<END>>")-1
            # check_range = self.T
            # check_range = len(words)

            # for t in range(check_range):
            #     one_hot = one_hot_seq[:,t]
            #     print np.argmax(one_hot)
            
            for t in range(check_range):
                one_hot = one_hot_seq[:, t]
                prob = self.softmax(probs[o][t])
                loss += self.cross_entropy_loss(prob, one_hot)
            # time average
            avg_losses.append(loss / (check_range * 1.))
            # batch average
            # avg_losses.append(loss / (len(probs) * 1.))
            
        print avg_losses
        return avg_losses


    def simple_avg_cross_entropy_loss(self, probs, query):
        words = query.split()
        # one_hot_seq = self.encode_one_hot(query)
        avg_losses = []
        captions = self.probs_to_captions(probs)
        for o in range(len(probs)):
            loss = 0
            caption = captions[o]
            # check_range = caption.split().index("<<END>>")-1
            # check_range = len(words)+1
            check_range = self.T
            for t in range(check_range):
                # one_hot = one_hot_seq[:, t]
                prob = self.softmax(probs[o][t])
                idx = self.get_idx(words[t]) if t < len(words) else self.get_idx("<<END>>")

                loss += (-math.log(prob[idx-1])) # if t < len(words) else 0.0 # (-math.log(prob[len(prob)-1]))

                # print self.get_token(idx-1), -math.log(prob[idx-1])
                # print self.get_token(idx-1), prob[idx-1], self.get_token(np.argmax(prob)), min(probs[o][t])# max(prob)

                # loss += self.cross_entropy_loss(prob, one_hot)
            avg_losses.append(loss / (check_range * 1.))
        print avg_losses
        return avg_losses
