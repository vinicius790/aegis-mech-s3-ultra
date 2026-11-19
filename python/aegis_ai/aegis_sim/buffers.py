# -*- coding: utf-8 -*-
"""Vectorized DelayBuffer & CircularBuffer (IsaacLab Sim2Real Buffers)."""

import torch


class DelayBuffer:
    """Simula atrasos variaveis de barramento (time_lags 0..history_len) e packet hold (hold_prob)."""

    def __init__(self, history_len: int, num_envs: int, dim: int, hold_prob: float = 0.0, device: str = "cpu"):
        self.history_len = max(1, int(history_len))
        self.num_envs = int(num_envs)
        self.dim = int(dim)
        self.hold_prob = float(hold_prob)
        self.device = device
        self._buf = torch.zeros((self.history_len + 1, self.num_envs, self.dim), device=device)
        self._lags = torch.zeros((self.num_envs,), dtype=torch.long, device=device)

    def reset(self, env_mask: torch.Tensor | None = None, lags: torch.Tensor | None = None):
        if env_mask is None:
            self._buf.zero_()
            if lags is not None:
                self._lags = lags.clamp(0, self.history_len)
            else:
                self._lags.zero_()
        else:
            self._buf[:, env_mask, :] = 0.0
            if lags is not None:
                self._lags[env_mask] = lags[env_mask].clamp(0, self.history_len)

    def compute(self, data: torch.Tensor) -> torch.Tensor:
        # Hold probability: congela entrada em porcentagem dos envs (I2C NACK / drop)
        if self.hold_prob > 0.0:
            keep_old = torch.rand(self.num_envs, device=self.device) < self.hold_prob
            newest = torch.where(keep_old.unsqueeze(-1), self._buf[0], data)
        else:
            newest = data

        self._buf = torch.roll(self._buf, shifts=1, dims=0)
        self._buf[0] = newest
        env_idx = torch.arange(self.num_envs, device=self.device)
        return self._buf[self._lags, env_idx, :]


class CircularBuffer:
    """Buffer circular vetorizado de observacoes históricas (zero-branch backfill no reset)."""

    def __init__(self, history_len: int, num_envs: int, dim: int, device: str = "cpu"):
        self.history_len = max(1, int(history_len))
        self.num_envs = int(num_envs)
        self.dim = int(dim)
        self.device = device
        self._buf = torch.zeros((self.num_envs, self.history_len, self.dim), device=device)
        self._is_fresh = torch.ones((self.num_envs,), dtype=torch.bool, device=device)

    def reset(self, env_mask: torch.Tensor | None = None):
        if env_mask is None:
            self._buf.zero_()
            self._is_fresh.fill_(True)
        else:
            self._buf[env_mask] = 0.0
            self._is_fresh[env_mask] = True

    def append(self, obs: torch.Tensor) -> torch.Tensor:
        if self._is_fresh.any():
            fresh_mask = self._is_fresh.unsqueeze(1).unsqueeze(2).expand(-1, self.history_len, self.dim)
            backfill = obs.unsqueeze(1).expand(-1, self.history_len, self.dim)
            self._buf = torch.where(fresh_mask, backfill, self._buf)
            self._is_fresh.fill_(False)
        else:
            self._buf = torch.roll(self._buf, shifts=-1, dims=1)
            self._buf[:, -1, :] = obs
        return self._buf.reshape(self.num_envs, self.history_len * self.dim)
