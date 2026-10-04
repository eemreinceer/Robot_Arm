#!/usr/bin/env python3
"""mjpeg_bridge sayfa üretimi — 2026-07-28 gösterim arızasının regresyonu.

ARIZA: sayfa `<img style='width:100%'>` kullanıyordu. 640x480 akış tarayıcı
penceresinin genişliğine gerdiriliyordu; 1080p ekranda 3x büyütülüp
bulanıklaşıyor ve "yayın çok büyük geliyor" izlenimi veriyordu. Oysa akış
ölçüldüğünde 640x480 / ~28 KB'lık karelerdi — sorun tamamen CSS'teydi.

Bu test ROS/cv2 gerektirmeyen saf HTML üretimini sınar.
"""
import importlib.util
import pathlib
from types import SimpleNamespace

import numpy as np
import pytest

MODULE_PATH = (pathlib.Path(__file__).resolve().parents[1]
               / "arm_perception" / "mjpeg_bridge.py")


def _load_index_html():
    """`index_html`'i ROS bağımlılıklarını import etmeden yükler.

    Modülün tepesinde rclpy/cv2 var; test ortamında bunlar olmayabilir.
    Fonksiyon saf string ürettiği için kaynaktan izole edip çalıştırıyoruz.
    """
    src = MODULE_PATH.read_text()
    start = src.index("def index_html(")
    end = src.index("class ThreadingHTTPServer")
    ns = {}
    exec(compile(src[start:end], str(MODULE_PATH), "exec"), ns)  # noqa: S102
    return ns["index_html"]


def test_goruntu_pencereye_gerdirilmiyor():
    """Çıplak `width:100%` geri gelirse bu test kırmızıya döner.

    `max-width:100%` MEŞRU (dar ekranda küçülme), çıplak `width:100%` ise
    arızanın kendisi. Bu yüzden negatif lookbehind ile ikisini ayırıyoruz —
    düz substring araması `max-width:100%`'e de takılır.
    """
    import re
    html = _load_index_html()().decode()
    assert "width:640px" in html, "görüntü doğal boyutunda sunulmalı"
    assert not re.search(r"(?<!max-)width:100%", html), (
        "çıplak width:100% görüntüyü pencereye gerer ve bulanıklaştırır; "
        "2026-07-28'de bu arıza yaşandı")


def test_kucuk_ekranda_tasmiyor():
    """Doğal boyut sabitlenirken dar ekranda yatay taşma olmamalı."""
    html = _load_index_html()().decode()
    assert "max-width:100%" in html
    assert "height:auto" in html


def test_genislik_parametrik():
    html = _load_index_html()(width_px=320).decode()
    assert "width:320px" in html


@pytest.mark.parametrize("path_fragment", ["/stream", "img src="])
def test_akis_kaynagi_sayfada(path_fragment):
    html = _load_index_html()().decode()
    assert path_fragment in html


class _FakePublisher:
    def __init__(self, subscriptions):
        self.subscriptions = subscriptions
        self.messages = []

    def get_subscription_count(self):
        return self.subscriptions

    def publish(self, message):
        self.messages.append(message)


class _FakeBridge:
    @staticmethod
    def imgmsg_to_cv2(_message, desired_encoding):
        assert desired_encoding == 'bgr8'
        return np.zeros((8, 8, 3), dtype=np.uint8)


@pytest.mark.parametrize('subscriptions,expected_messages', [(0, 0), (1, 1)])
def test_compressed_ros_publish_is_gated_by_real_subscribers(
        subscriptions, expected_messages):
    from arm_perception import mjpeg_bridge

    publisher = _FakePublisher(subscriptions)
    node = SimpleNamespace(
        bridge=_FakeBridge(), quality=80, compressed_publisher=publisher)
    message = SimpleNamespace(header=SimpleNamespace())

    mjpeg_bridge.MjpegBridge._on_image(node, message)

    assert len(publisher.messages) == expected_messages
