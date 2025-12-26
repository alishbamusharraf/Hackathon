import { useEffect } from 'react';
import Layout from '@theme/Layout';
import lottie from 'lottie-web';
import robotData from '../assets/robot.json'; // apni robot JSON

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
      <main style={{ fontFamily: "'Inter', sans-serif", minHeight: '100vh', overflowX: 'hidden' }}>
        
        {/* HERO SECTION */}
        <section style={{
          display: 'flex', 
          alignItems: 'center', 
          justifyContent: 'space-between', 
          padding: '100px 10%', 
          gap: '50px',
          position: 'relative',
          background: 'linear-gradient(135deg, #ede9fe 0%, #ddd6fe 100%)',
          overflow: 'hidden'
        }}>
          
          {/* LEFT: Heading + Text + Buttons */}
          <div style={{ flex: 1, textAlign: 'left', position: 'relative', zIndex: 1 }}>
            <h1 style={{ fontSize: '3.5rem', fontWeight: '900', marginBottom: '20px', background: 'linear-gradient(90deg, #7c3aed, #9333ea)', WebkitBackgroundClip: 'text', color: 'transparent' }}>
              Physical AI & <br /> Humanoid Robotics
            </h1>
            <p style={{ fontSize: '1.25rem', color: '#5b21b6', marginBottom: '40px', maxWidth: '500px' }}>
              Step-by-step labs and examples covering ROS 2, NVIDIA Isaac, digital twins, and Vision-Language-Action systems.
            </p>

            <div style={{ display: 'flex', gap: '20px', flexWrap: 'wrap' }}>
              <button className="primary-btn">📘 Read Book</button>
              <button className="secondary-btn">🚀 Start Learning</button>
            </div>
          </div>

          {/* RIGHT: Lottie Robot */}
          <div style={{ flex: 1, textAlign: 'center', position: 'relative', zIndex: 1 }}>
            <div id="robot-animation" style={{ width: '400px', height: '400px', margin: '0 auto' }} />
          </div>
        </section>

        {/* WHAT YOU LEARN SECTION */}
        <section style={{ padding: '80px 10%', textAlign: 'center', background: '#f3e8ff' }}>
          <h2 style={{ fontSize: '2.5rem', fontWeight: '700', marginBottom: '50px', color: '#7c3aed' }}>What You'll Learn</h2>
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(220px, 1fr))', gap: '30px', maxWidth: '1000px', margin: '0 auto' }}>
            <FeatureCard title="🤖 Humanoid Robots" description="Design and control lifelike humanoid robots." />
            <FeatureCard title="🧠 Physical AI" description="Intelligent agents interacting with the real world." />
            <FeatureCard title="⚙️ ROS 2" description="Robot Operating System for modern robotics." />
            <FeatureCard title="🦾 Sensors & Actuators" description="Integrate sensors for perception and control." />
          </div>
        </section>

        {/* EXTRA SECTION */}
        <section style={{ padding: '80px 10%', textAlign: 'center', background: '#ede9fe' }}>
          <h2 style={{ fontSize: '2.5rem', fontWeight: '700', marginBottom: '30px', color: '#7c3aed' }}>Robotics in Action</h2>
          <p style={{ fontSize: '1.2rem', color: '#5b21b6', marginBottom: '40px' }}>
            Explore simulations and interactive examples to see humanoid robotics in real-world scenarios.
          </p>
          <button className="primary-btn">Try Simulation</button>
        </section>

        {/* GLOBAL STYLES */}
        <style>{`
          .primary-btn {
            padding: 16px 32px;
            border-radius: 12px;
            font-size: 1.1rem;
            font-weight: 600;
            cursor: pointer;
            background-color: #7c3aed;
            color: white;
            border: none;
            transition: all 0.3s ease;
          }
          .primary-btn:hover { transform: scale(1.07); box-shadow: 0 8px 24px rgba(124,58,237,0.35); }

          .secondary-btn {
            padding: 16px 32px;
            border-radius: 12px;
            font-size: 1.1rem;
            font-weight: 600;
            cursor: pointer;
            background-color: transparent;
            color: #7c3aed;
            border: 2px solid #7c3aed;
            transition: all 0.3s ease;
          }
          .secondary-btn:hover { background-color: #ede9fe; transform: scale(1.07); border-color: #9333ea; }
        `}</style>
      </main>
    </Layout>
  );
}

function FeatureCard({ title, description }) {
  return (
    <div
      style={{
        padding: '25px',
        borderRadius: '12px',
        backgroundColor: '#faf5ff',
        fontWeight: '600',
        fontSize: '1.1rem',
        textAlign: 'center',
        boxShadow: '0 6px 18px rgba(124,58,237,0.15)',
        transition: 'all 0.3s ease',
        cursor: 'pointer',
      }}
      onMouseOver={(e) => { e.currentTarget.style.transform = 'translateY(-10px)'; e.currentTarget.style.boxShadow = '0 12px 25px rgba(124,58,237,0.25)'; }}
      onMouseOut={(e) => { e.currentTarget.style.transform = 'translateY(0)'; e.currentTarget.style.boxShadow = '0 6px 18px rgba(124,58,237,0.15)'; }}
    >
      <h3 style={{ fontSize: '1.4rem', marginBottom: '12px', color: '#7c3aed' }}>{title}</h3>
      <p style={{ fontSize: '1rem', color: '#5b21b6', marginBottom: '15px' }}>{description}</p>
      <button
        style={{
          padding: '8px 20px',
          borderRadius: '10px',
          border: 'none',
          backgroundColor: '#9333ea',
          color: '#fff',
          fontWeight: '600',
          cursor: 'pointer',
          transition: 'all 0.3s ease',
        }}
        onMouseOver={(e) => (e.currentTarget.style.transform = 'scale(1.05)')}
        onMouseOut={(e) => (e.currentTarget.style.transform = 'scale(1)')}
      >
        Learn More
      </button>
    </div>
  );
}
