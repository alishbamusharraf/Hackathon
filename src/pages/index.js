import { useEffect } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import lottie from 'lottie-web';
import robotData from '../assets/robot.json';

export default function Home() {
  useEffect(() => {
    if (typeof window !== 'undefined') {
      lottie.loadAnimation({
        container: document.getElementById('robot-animation'),
        renderer: 'svg',
        loop: true,
        autoplay: true,
        animationData: robotData,
      });
    }
  }, []);

  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Learn ROS 2, NVIDIA Isaac, Digital Twins & Vision-Language-Action systems"
    >
      <main style={{ fontFamily: "'Inter', sans-serif", overflowX: 'hidden' }}>

        {/* HERO SECTION */}
        <section style={{
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          padding: '100px 10%',
          gap: '60px',
          background: 'radial-gradient(circle at top right, #0f172a, #1e293b)'
        }}>
          {/* LEFT */}
          <div style={{ flex: 1 }}>
            <h1 style={{
              fontSize: '3.6rem',
              fontWeight: 900,
              marginBottom: '24px',
              background: 'linear-gradient(90deg, #06b6d4, #3b82f6)',
              WebkitBackgroundClip: 'text',
              color: 'transparent'
            }}>
              Physical AI & <br /> Humanoid Robotics
            </h1>

            <p style={{
              fontSize: '1.25rem',
              color: '#cbd5e1',
              marginBottom: '40px',
              maxWidth: '520px',
              lineHeight: 1.7
            }}>
              Learn how intelligent humanoid robots perceive, reason, and act
              in the physical world using modern AI systems.
            </p>

            <div style={{ display: 'flex', gap: '20px' }}>
              <Link to="/docs/intro" className="primary-btn">📘 Read Book</Link>
              <Link to="/docs/getting-started" className="secondary-btn">🚀 Start Learning</Link>
            </div>
          </div>

          {/* RIGHT */}
          <div style={{ flex: 1, textAlign: 'center' }}>
            <div
              id="robot-animation"
              style={{
                width: '420px',
                height: '420px',
                margin: '0 auto',
                filter: 'drop-shadow(0 25px 45px rgba(3,105,161,0.45))'
              }}
            />
          </div>
        </section>

        {/* WHAT YOU'LL LEARN – ZIG ZAG */}
        <section style={{ padding: '100px 10%', background: '#0f172a' }}>
          <h2 style={{
            fontSize: '2.7rem',
            fontWeight: 800,
            marginBottom: '70px',
            textAlign: 'center',
            color: '#06b6d4'
          }}>
            What You'll Learn
          </h2>

          <div style={{ maxWidth: '1100px', margin: '0 auto' }}>
            <ZigCard
              align="left"
              title="🤖 Humanoid Robots"
              text="Build and control realistic humanoid robots with motion, balance, and intelligence."
            />
            <ZigCard
              align="right"
              title="🧠 Physical AI"
              text="Create AI systems that sense, reason, and act in the real physical environment."
            />
            <ZigCard
              align="left"
              title="⚙️ ROS 2"
              text="Master the industry-standard robotics middleware used in real robots."
            />
            <ZigCard
              align="right"
              title="🦾 Sensors & Control"
              text="Understand perception pipelines, sensors, actuators, and control systems."
            />
          </div>
        </section>

        {/* ROBOTICS INFO */}
        <section style={{ padding: '100px 10%', background: '#1e293b' }}>
          <div style={{
            maxWidth: '1000px',
            margin: '0 auto',
            background: '#0f172a',
            padding: '60px',
            borderRadius: '28px',
            boxShadow: '0 15px 40px rgba(3,105,161,0.3)',
            textAlign: 'center',
            transition: '0.3s'
          }}
            className="robotics-info-box"
          >
            <h2 style={{
              fontSize: '2.5rem',
              marginBottom: '28px',
              color: '#06b6d4',
              fontWeight: 700
            }}>
              🤖 Robotics Beyond Theory
            </h2>

            <p style={{
              fontSize: '1.2rem',
              color: '#cbd5e1',
              lineHeight: 1.8,
              maxWidth: '850px',
              margin: '0 auto'
            }}>
              Dive deeper into real humanoid intelligence — blending perception,
              AI reasoning, and physical interaction. This section highlights
              practical robotics applications, giving you insights into how
              advanced robots see, think, and act in real environments.
            </p>
          </div>

          <style>{`
            .robotics-info-box:hover {
              transform: translateY(-6px) scale(1.02);
              box-shadow: 0 25px 60px rgba(3,105,161,0.4);
            }
          `}</style>
        </section>

        {/* BUTTON STYLES */}
        <style>{`
          .primary-btn {
            padding: 16px 34px;
            border-radius: 14px;
            font-size: 1.1rem;
            font-weight: 600;
            background: #06b6d4;
            color: white;
            text-decoration: none;
            transition: 0.35s;
          }
          .primary-btn:hover {
            transform: translateY(-4px) scale(1.05);
            box-shadow: 0 12px 32px rgba(3,105,161,0.45);
          }

          .secondary-btn {
            padding: 16px 34px;
            border-radius: 14px;
            font-size: 1.1rem;
            font-weight: 600;
            border: 2px solid #06b6d4;
            color: #06b6d4;
            text-decoration: none;
            transition: 0.35s;
          }
          .secondary-btn:hover {
            background: #0f172a;
            color: #06b6d4;
            transform: translateY(-4px) scale(1.05);
          }
        `}</style>
      </main>
    </Layout>
  );
}

/* ZIG ZAG CARD */
function ZigCard({ align, title, text }) {
  return (
    <div
      style={{
        display: 'flex',
        justifyContent: align === 'left' ? 'flex-start' : 'flex-end',
        marginBottom: '55px',
      }}
    >
      <div
        className="zig-card"
        style={{
          width: '420px',
          maxWidth: '100%',
          padding: '32px',
          borderRadius: '22px',
          background: '#0f172a',
          boxShadow: '0 12px 30px rgba(3,105,161,0.25)',
          transition: '0.35s',
        }}
      >
        <h3 style={{ color: '#06b6d4', marginBottom: '14px', fontSize: '1.5rem' }}>
          {title}
        </h3>

        <p style={{ color: '#cbd5e1', fontSize: '1.05rem', lineHeight: 1.8 }}>
          {text}
        </p>
      </div>

      <style>{`
        .zig-card:hover {
          transform: translateY(-10px) scale(1.04);
          box-shadow: 0 24px 55px rgba(3,105,161,0.4);
        }
      `}</style>
    </div>
  );
}
