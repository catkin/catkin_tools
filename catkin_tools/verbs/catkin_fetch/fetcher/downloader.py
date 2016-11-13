import logging
import subprocess

from os import path
from subprocess import PIPE

logging.basicConfig()


class Downloader(object):
    """Downloader for dependency dict."""

    GIT_CHECK_CMD_MASK = "git ls-remote {url}"
    GIT_CLONE_CMD_MASK = "git clone --recursive {url} {path}"


    def __init__(self, ws_path, available_pkgs, log_level="INFO"):
        super(Downloader, self).__init__()
        self.ws_path = ws_path
        self.available_pkgs = available_pkgs
        self.log = logging.getLogger('Downloader')
        self.log.setLevel(logging.getLevelName(log_level))

    def download_dependencies(self, dep_dict):
        checked_deps = self.check_dependencies(dep_dict)
        self.clone_dependencies(checked_deps)

    def clone_dependencies(self, checked_deps):
        self.log.info(" Cloning valid dependencies:")
        for name, url in checked_deps.items():
            if name in self.available_pkgs:
                self.log.info(" [%-21s]: already exists", name)
                continue
            dep_path = path.join(self.ws_path, name)
            cmd_clone = Downloader.GIT_CLONE_CMD_MASK.format(
                            url=url, path=dep_path)
            try:
                subprocess.check_output(cmd_clone,
                                        stderr=subprocess.STDOUT,
                                        shell=True)
                self.log.info("  %-21s: cloned", "[" + name + "]")
            except subprocess.CalledProcessError as e:
                out_str = e.output.decode("utf8")
                self.log.debug(" Output: %s", out_str)
                if "already exists" in out_str:
                    self.log.info(" [%-21s]: already exists", name)
                else:
                    self.log.error(" [%-20s]: error. Git error code: %s",
                                   name, e.returncode)
        self.log.info(" Dependencies cloned.\n")

    def check_dependencies(self, dep_dict):
        self.log.info(" Checking dependencies:")
        checked_deps = {}
        for name, url in dep_dict.items():
            if Downloader.repository_exists(url):
                self.log.info(" [%-21s]: %s", name, url)
                checked_deps[name] = url
            else:
                self.log.info(" [%-21s]: not found", name)
        self.log.info(" Check completed.")
        return checked_deps

    @staticmethod
    def repository_exists(url):
        git_cmd = Downloader.GIT_CHECK_CMD_MASK.format(url=url)
        try:
            subprocess.check_call(git_cmd, stdout=PIPE, stderr=PIPE, shell=True)
            return True
        except subprocess.CalledProcessError:
            return False
